"""ROS 2 node for finite mobile-base next-best-view planning."""

from __future__ import annotations

from collections import Counter
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .nbv import (
    evaluate_base_candidates,
    generate_base_candidates,
    NBVConfig,
    OccupancyGrid2D,
)
from .target_belief_node import position_covariance
from .visibility import PinholeCamera, VisibilityConfig


def quaternion_matrix(quaternion) -> np.ndarray:
    """Return a 3x3 rotation matrix for a geometry quaternion."""
    values = np.array(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w), dtype=float
    )
    norm = float(values @ values)
    if norm <= 1.0e-16 or not math.isfinite(norm):
        raise ValueError("quaternion must be finite and non-zero")
    x, y, z, w = values / math.sqrt(norm)
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        )
    )


def transform_matrix(transform) -> np.ndarray:
    """Convert geometry Transform to a homogeneous matrix."""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(transform.rotation)
    matrix[:3, 3] = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    return matrix


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a geometry quaternion."""
    rotation = quaternion_matrix(quaternion)
    return math.atan2(rotation[1, 0], rotation[0, 0])


def yaw_quaternion(yaw: float) -> Quaternion:
    """Create a normalized planar-yaw quaternion."""
    return Quaternion(z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw))


def path_length(path: Path) -> float:
    """Return translational length of a Nav2 path."""
    total = 0.0
    for first, second in zip(path.poses, path.poses[1:]):
        total += math.hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y,
        )
    return total


def path_minimum_xy_distance(path: Path, point_xy) -> float:
    """Return the minimum planar distance from a Nav2 path to a point."""
    if not path.poses:
        return math.inf
    target_x, target_y = point_xy
    return min(
        math.hypot(
            pose.pose.position.x - target_x,
            pose.pose.position.y - target_y,
        )
        for pose in path.poses
    )


def effective_path_target_clearance(
    configured_clearance_m: float,
    initial_clearance_m: float,
    start_tolerance_m: float,
) -> float:
    """Allow a robot already inside the nominal radius to safely move outward."""
    values = (configured_clearance_m, initial_clearance_m, start_tolerance_m)
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError("path target clearances must be finite and non-negative")
    return min(configured_clearance_m, max(0.0, initial_clearance_m - start_tolerance_m))


class NBVPlannerNode(Node):
    """Generate, filter and query Nav2 for fixed-camera base viewpoints."""

    def __init__(self) -> None:
        super().__init__("nbv_planner")
        defaults = {
            "target_detection_topic": "/active_perception/selected_target_detection",
            "target_belief_topic": "/active_perception/target_belief",
            "target_id_topic": "/active_perception/selected_target_id",
            "map_topic": "/map",
            "camera_info_topic": "/sensors/nav_camera/color/camera_info",
            "best_pose_topic": "/active_perception/best_view_pose",
            "candidate_poses_topic": "/active_perception/candidate_view_poses",
            "markers_topic": "/active_perception/candidate_view_markers",
            "diagnostics_topic": "/active_perception/nbv_diagnostics",
            "compute_path_action": "/compute_path_to_pose",
            "map_frame": "map",
            "base_frame": "base_link",
            "camera_frame": "nav_camera_color_optical_frame",
            "radii_m": [1.2, 1.4, 1.6],
            "samples_per_ring": 12,
            "robot_radius_m": 0.32,
            "safety_margin_m": 0.08,
            "occupied_threshold": 50,
            "unknown_is_occupied": True,
            "target_ray_exclusion_m": 0.15,
            "desired_clearance_m": 0.60,
            "max_path_length_m": 5.0,
            "planner_id": "GridBased",
            "action_server_timeout_seconds": 1.0,
            "path_query_timeout_seconds": 8.0,
            "minimum_path_target_clearance_m": 1.1,
            "path_clearance_start_tolerance_m": 0.05,
            "transform_timeout_seconds": 0.2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_detection: Detection3DArray | None = None
        self.target_belief = None
        self.target_id = ""
        self.map_message: OccupancyGrid | None = None
        self.camera_info: CameraInfo | None = None
        self.busy = False
        self.pending_candidates = []
        self.path_lengths: dict[int, float | None] = {}
        self.current_query = 0
        self.query_token = 0
        self.active_candidate_index: int | None = None
        self.active_goal_handle = None
        self.query_started_monotonic = 0.0
        self.current_base_pose = None
        self.target_world = None
        self.target_size_world = None
        self.prior_covariance = None
        self.camera_model = None
        self.camera_from_base = None
        self.grid = None
        self.nbv_config = None

        self.create_subscription(
            Detection3DArray,
            str(self.get_parameter("target_detection_topic").value),
            self._target_detection_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("target_belief_topic").value),
            self._target_belief_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_id_topic").value),
            self._target_id_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            transient_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.best_pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("best_pose_topic").value), 10
        )
        self.candidate_poses_pub = self.create_publisher(
            PoseArray, str(self.get_parameter("candidate_poses_topic").value), 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("markers_topic").value), 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter("compute_path_action").value),
        )
        self.plan_service = self.create_service(
            Trigger, "/active_perception/compute_next_best_view", self._start_planning
        )
        self.query_watchdog = self.create_timer(0.1, self._monitor_query_timeout)

    def _target_detection_callback(self, message) -> None:
        self.target_detection = message if message.detections else None

    def _target_belief_callback(self, message) -> None:
        self.target_belief = message

    def _target_id_callback(self, message: String) -> None:
        new_id = message.data.strip()
        if new_id != self.target_id:
            self.target_id = new_id
            self.target_belief = None

    def _map_callback(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _lookup_matrix(self, target_frame: str, source_frame: str) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(
                seconds=float(self.get_parameter("transform_timeout_seconds").value)
            ),
        )
        return transform_matrix(transform.transform)

    def _prepare_problem(self) -> tuple[bool, str]:
        missing = []
        if self.target_detection is None:
            missing.append("selected_target_detection")
        if self.target_belief is None:
            missing.append("target_belief")
        if self.map_message is None:
            missing.append("map")
        if self.camera_info is None:
            missing.append("camera_info")
        if missing:
            return False, "missing: " + ", ".join(missing)
        assert self.target_detection is not None
        assert self.target_belief is not None
        assert self.map_message is not None
        assert self.camera_info is not None
        if not self.target_detection.detections:
            return False, "selected target detection is empty"
        detection = self.target_detection.detections[0]
        if self.target_id and detection.id != self.target_id:
            return False, "selected target ID and detection do not match"
        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        camera_frame = str(self.get_parameter("camera_frame").value)
        try:
            map_from_base = self._lookup_matrix(map_frame, base_frame)
            self.camera_from_base = self._lookup_matrix(camera_frame, base_frame)
            belief_frame = self.target_belief.header.frame_id
            map_from_belief = (
                np.eye(4)
                if belief_frame == map_frame
                else self._lookup_matrix(map_frame, belief_frame)
            )
        except TransformException as error:
            return False, f"TF unavailable: {error}"

        belief_position = self.target_belief.pose.pose.position
        self.target_world = (
            map_from_belief
            @ np.array((belief_position.x, belief_position.y, belief_position.z, 1.0))
        )[:3]
        belief_covariance = position_covariance(self.target_belief)
        rotation = map_from_belief[:3, :3]
        self.prior_covariance = rotation @ belief_covariance @ rotation.T
        detection_size = detection.bbox.size
        size = np.array((detection_size.x, detection_size.y, detection_size.z))
        if np.any(size <= 0.0) or not np.all(np.isfinite(size)):
            return False, "selected target has invalid 3-D size"
        detection_frame = self.target_detection.header.frame_id
        try:
            map_from_detection = (
                np.eye(4)
                if detection_frame == map_frame
                else self._lookup_matrix(map_frame, detection_frame)
            )
        except TransformException as error:
            return False, f"target-size TF unavailable: {error}"
        self.target_size_world = np.abs(map_from_detection[:3, :3]) @ size
        self.current_base_pose = np.array(
            (
                map_from_base[0, 3],
                map_from_base[1, 3],
                math.atan2(map_from_base[1, 0], map_from_base[0, 0]),
            )
        )
        origin = self.map_message.info.origin
        grid_data = np.asarray(self.map_message.data, dtype=np.int16).reshape(
            self.map_message.info.height, self.map_message.info.width
        )
        self.grid = OccupancyGrid2D(
            grid_data,
            self.map_message.info.resolution,
            origin.position.x,
            origin.position.y,
            yaw_from_quaternion(origin.orientation),
        )
        self.camera_model = PinholeCamera(
            self.camera_info.width,
            self.camera_info.height,
            self.camera_info.k[0],
            self.camera_info.k[4],
            self.camera_info.k[2],
            self.camera_info.k[5],
        )
        self.nbv_config = NBVConfig(
            radii_m=tuple(float(value) for value in self.get_parameter("radii_m").value),
            samples_per_ring=int(self.get_parameter("samples_per_ring").value),
            robot_radius_m=float(self.get_parameter("robot_radius_m").value),
            safety_margin_m=float(self.get_parameter("safety_margin_m").value),
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
            unknown_is_occupied=bool(self.get_parameter("unknown_is_occupied").value),
            target_ray_exclusion_m=float(
                self.get_parameter("target_ray_exclusion_m").value
            ),
            desired_clearance_m=float(self.get_parameter("desired_clearance_m").value),
            max_path_length_m=float(self.get_parameter("max_path_length_m").value),
        )
        return True, "ready"

    def _start_planning(self, _request: Trigger.Request, response: Trigger.Response):
        if self.busy:
            response.success = False
            response.message = "NBV planning already in progress"
            return response
        ready, message = self._prepare_problem()
        if not ready:
            response.success = False
            response.message = message
            return response
        timeout = float(self.get_parameter("action_server_timeout_seconds").value)
        if not self.compute_path_client.wait_for_server(timeout_sec=timeout):
            response.success = False
            response.message = "Nav2 ComputePathToPose action is unavailable"
            return response
        assert self.target_world is not None
        assert self.nbv_config is not None
        candidates = generate_base_candidates(
            self.target_world[:2],
            radii_m=self.nbv_config.radii_m,
            samples_per_ring=self.nbv_config.samples_per_ring,
        )
        preliminary = self._evaluate(candidates, None)
        self.pending_candidates = [
            score.candidate
            for score in preliminary
            if score.footprint_free and score.visibility.visible
        ]
        self.path_lengths = {}
        self.current_query = 0
        if not self.pending_candidates:
            self._publish_results(preliminary)
            reasons = Counter(
                reason for score in preliminary for reason in score.rejection_reasons
            )
            summary = ", ".join(
                f"{reason}={count}" for reason, count in sorted(reasons.items())
            ) or "no rejection reason reported"
            free_count = sum(score.footprint_free for score in preliminary)
            visible_count = sum(score.visibility.visible for score in preliminary)
            detail = (
                f"no feasible preliminary candidates: total={len(preliminary)}, "
                f"footprint_free={free_count}, visible={visible_count}; {summary}"
            )
            self.get_logger().warn(detail)
            response.success = False
            response.message = detail
            return response
        self.busy = True
        self._query_next_candidate()
        response.success = True
        response.message = f"querying Nav2 for {len(self.pending_candidates)} candidates"
        return response

    def _evaluate(self, candidates, path_lengths):
        return evaluate_base_candidates(
            candidates,
            current_base_pose=self.current_base_pose,
            target_world=self.target_world,
            target_size_world=self.target_size_world,
            prior_covariance=self.prior_covariance,
            camera=self.camera_model,
            camera_from_base=self.camera_from_base,
            occupancy_grid=self.grid,
            config=self.nbv_config,
            visibility_config=VisibilityConfig(),
            path_lengths_m=path_lengths,
        )

    def _query_next_candidate(self) -> None:
        if self.current_query >= len(self.pending_candidates):
            self.busy = False
            self.query_token += 1
            self.active_candidate_index = None
            self.active_goal_handle = None
            self._publish_results(self._evaluate(self.pending_candidates, self.path_lengths))
            return
        candidate = self.pending_candidates[self.current_query]
        self.query_token += 1
        token = self.query_token
        self.active_candidate_index = candidate.index
        self.active_goal_handle = None
        self.query_started_monotonic = time.monotonic()
        goal = ComputePathToPose.Goal()
        goal.goal = self._candidate_pose(candidate)
        goal.planner_id = str(self.get_parameter("planner_id").value)
        goal.use_start = False
        future = self.compute_path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, index=candidate.index, query_token=token: self._goal_response(
                completed, index, query_token
            )
        )

    def _goal_response(self, future, candidate_index: int, query_token: int) -> None:
        if not self.busy or query_token != self.query_token:
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._complete_query(candidate_index, None, query_token)
                return
            self.active_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed, index=candidate_index, token=query_token: self._path_result(
                    completed, index, token
                )
            )
        except Exception as error:
            self.get_logger().error(f"Nav2 goal response failed: {error}")
            self._complete_query(candidate_index, None, query_token)

    def _path_result(self, future, candidate_index: int, query_token: int) -> None:
        if not self.busy or query_token != self.query_token:
            return
        length = None
        try:
            wrapped = future.result()
            result = wrapped.result
            if (
                result.error_code == ComputePathToPose.Result.NONE
                and len(result.path.poses) >= 2
            ):
                assert self.target_world is not None
                clearance = path_minimum_xy_distance(
                    result.path, self.target_world[:2]
                )
                minimum_clearance = float(
                    self.get_parameter("minimum_path_target_clearance_m").value
                )
                assert self.current_base_pose is not None
                initial_clearance = math.hypot(
                    self.current_base_pose[0] - self.target_world[0],
                    self.current_base_pose[1] - self.target_world[1],
                )
                effective_clearance = effective_path_target_clearance(
                    minimum_clearance,
                    initial_clearance,
                    float(self.get_parameter("path_clearance_start_tolerance_m").value),
                )
                if clearance >= effective_clearance:
                    length = path_length(result.path)
                else:
                    self.get_logger().warn(
                        f"Rejecting candidate {candidate_index}: path passes "
                        f"{clearance:.3f} m from target, below "
                        f"effective limit {effective_clearance:.3f} m "
                        f"(configured={minimum_clearance:.3f}, "
                        f"start={initial_clearance:.3f})"
                    )
        except Exception as error:
            self.get_logger().error(f"Nav2 path result failed: {error}")
        self._complete_query(candidate_index, length, query_token)

    def _complete_query(
        self, candidate_index: int, length: float | None, query_token: int
    ) -> None:
        if not self.busy or query_token != self.query_token:
            return
        self.path_lengths[candidate_index] = length
        self.active_candidate_index = None
        self.active_goal_handle = None
        self._advance_query()

    def _monitor_query_timeout(self) -> None:
        if not self.busy or self.active_candidate_index is None:
            return
        timeout = float(self.get_parameter("path_query_timeout_seconds").value)
        if timeout <= 0.0:
            return
        elapsed = time.monotonic() - self.query_started_monotonic
        if elapsed < timeout:
            return
        candidate_index = self.active_candidate_index
        goal_handle = self.active_goal_handle
        self.query_token += 1  # Invalidate any late action callbacks.
        self.path_lengths[candidate_index] = None
        self.active_candidate_index = None
        self.active_goal_handle = None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self.get_logger().warn(
            f"Nav2 path query timed out for candidate {candidate_index} after "
            f"{elapsed:.1f} s; marking it unreachable"
        )
        self._advance_query()

    def _advance_query(self) -> None:
        self.current_query += 1
        self._query_next_candidate()

    def _candidate_pose(self, candidate) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        message.pose.position.x = candidate.x
        message.pose.position.y = candidate.y
        message.pose.orientation = yaw_quaternion(candidate.yaw)
        return message

    def _publish_results(self, scores) -> None:
        stamp = self.get_clock().now().to_msg()
        frame = str(self.get_parameter("map_frame").value)
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = frame
        markers = MarkerArray()
        feasible = [score for score in scores if math.isfinite(score.total_score)]
        best = feasible[0] if feasible else None
        for marker_id, score in enumerate(scores):
            pose = self._candidate_pose(score.candidate).pose
            pose_array.poses.append(pose)
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = "nbv_candidates"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.20
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            marker.color.a = 0.9
            if not math.isfinite(score.total_score):
                marker.color.r = 0.7
                marker.color.g = 0.1
            elif best is score:
                marker.color.g = 1.0
            else:
                marker.color.r = 0.9
                marker.color.g = 0.7
            markers.markers.append(marker)
        self.candidate_poses_pub.publish(pose_array)
        self.markers_pub.publish(markers)
        diagnostic = DiagnosticStatus()
        diagnostic.name = "active_perception/nbv_planner"
        diagnostic.hardware_id = "software"
        if best is None:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = "no_feasible_candidate"
        else:
            diagnostic.level = DiagnosticStatus.OK
            diagnostic.message = "best_candidate_ready"
            best_pose = self._candidate_pose(best.candidate)
            self.best_pose_pub.publish(best_pose)
            values = {
                "target_id": self.target_id,
                "candidate_index": best.candidate.index,
                "total_score": best.total_score,
                "information_gain_nats": best.information_gain_term,
                "visible_fraction": best.visible_fraction_term,
                "clearance_m": best.clearance_m,
                "map_occlusion_fraction": best.map_occlusion_fraction,
                "path_length_m": best.path_length_m,
                "yaw_change_rad": best.yaw_change_rad,
                "path_source": best.path_source,
                "feasible_candidates": len(feasible),
                "total_candidates": len(scores),
            }
            diagnostic.values = [
                KeyValue(key=key, value=str(value)) for key, value in values.items()
            ]
            self.get_logger().info(
                f"NBV best index={best.candidate.index}, score={best.total_score:.4f}, "
                f"path={best.path_length_m:.3f} m, gain={best.information_gain_term:.4f} nats"
            )
        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status.append(diagnostic)
        self.diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NBVPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
