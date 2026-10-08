"""ROS 2 node producing a standard 3D observation from aligned RGB-D."""

from __future__ import annotations

import math

from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from target_perception.color_depth import (
    CameraIntrinsics,
    TargetEstimate,
    depth_to_meters,
    estimate_color_target,
)


def _key_value(key: str, value: object) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = str(value)
    return item


def rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_covariance(
    estimate: TargetEstimate,
    rotation: np.ndarray,
) -> list[float]:
    source = np.diag(np.asarray(estimate.position_variance, dtype=np.float64))
    target = rotation @ source @ rotation.T
    covariance = [0.0] * 36
    for row in range(3):
        for column in range(3):
            covariance[row * 6 + column] = float(target[row, column])
    # This baseline estimates position only; a large finite value explicitly
    # marks target orientation as unknown.
    covariance[21] = 1.0e3
    covariance[28] = 1.0e3
    covariance[35] = 1.0e3
    return covariance


class ColorDepthDetector(Node):
    """Detect one configured color region and deproject aligned depth points."""

    def __init__(self) -> None:
        super().__init__("color_depth_detector")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.output_frame = str(self.get_parameter("output_frame").value).strip()
        self.target_id = str(self.get_parameter("target_id").value)
        self.class_id = str(self.get_parameter("class_id").value)
        self.hsv_lower = self._hsv_parameter("hsv_lower")
        self.hsv_upper = self._hsv_parameter("hsv_upper")
        self.depth_scale = float(self.get_parameter("depth_scale_16u").value)
        self.transform_timeout = float(
            self.get_parameter("transform_timeout_s").value
        )
        self.publish_debug_mask = bool(
            self.get_parameter("publish_debug_mask").value
        )

        self.detections_3d_pub = self.create_publisher(
            Detection3DArray,
            str(self.get_parameter("detections_3d_topic").value),
            qos_profile_sensor_data,
        )
        self.detections_2d_pub = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter("detections_2d_topic").value),
            qos_profile_sensor_data,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("target_pose_topic").value),
            qos_profile_sensor_data,
        )
        self.metrics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("metrics_topic").value),
            qos_profile_sensor_data,
        )
        self.debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_mask_topic").value),
            qos_profile_sensor_data,
        )

        self.color_sub = Subscriber(
            self,
            Image,
            str(self.get_parameter("color_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.info_sub = Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_s").value),
        )
        self.sync.registerCallback(self._callback)
        self.get_logger().info(
            "RGB-D target detector ready: "
            f"class={self.class_id}, output_frame={self.output_frame or '<input>'}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "color_topic": "/sensors/nav_camera/color/image_raw",
            "depth_topic": "/sensors/nav_camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/sensors/nav_camera/color/camera_info",
            "detections_3d_topic": "/target_perception/detections_3d",
            "detections_2d_topic": "/target_perception/detections_2d",
            "target_pose_topic": "/target_perception/target_pose",
            "metrics_topic": "/target_perception/metrics",
            "debug_mask_topic": "/target_perception/debug/mask",
            "output_frame": "base_link",
            "target_id": "green_target_0",
            "class_id": "green_target",
            "hsv_lower": [35, 80, 50],
            "hsv_upper": [90, 255, 255],
            "morphology_kernel": 5,
            "min_contour_area": 200.0,
            "min_depth_points": 40,
            "min_depth_m": 0.20,
            "max_depth_m": 3.0,
            "depth_scale_16u": 0.001,
            "sync_queue_size": 10,
            "sync_slop_s": 0.05,
            "transform_timeout_s": 0.10,
            "publish_debug_mask": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _hsv_parameter(self, name: str) -> tuple[int, int, int]:
        values = tuple(int(value) for value in self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly three integers")
        if not 0 <= values[0] <= 179 or any(not 0 <= item <= 255 for item in values[1:]):
            raise ValueError(f"{name} is outside OpenCV HSV limits")
        return values

    def _callback(
        self,
        color_message: Image,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(
                depth_message,
                desired_encoding="passthrough",
            )
            depth_m = depth_to_meters(
                np.asarray(raw_depth),
                depth_message.encoding,
                self.depth_scale,
            )
            intrinsics = CameraIntrinsics(
                fx=float(camera_info.k[0]),
                fy=float(camera_info.k[4]),
                cx=float(camera_info.k[2]),
                cy=float(camera_info.k[5]),
            )
            estimate = estimate_color_target(
                np.asarray(bgr),
                depth_m,
                intrinsics,
                hsv_lower=self.hsv_lower,
                hsv_upper=self.hsv_upper,
                min_contour_area=float(
                    self.get_parameter("min_contour_area").value
                ),
                min_depth_points=int(
                    self.get_parameter("min_depth_points").value
                ),
                min_depth_m=float(self.get_parameter("min_depth_m").value),
                max_depth_m=float(self.get_parameter("max_depth_m").value),
                morphology_kernel=int(
                    self.get_parameter("morphology_kernel").value
                ),
            )
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(
                f"RGB-D input rejected: {error}", throttle_duration_sec=2.0
            )
            self._publish_metrics(
                color_message,
                DiagnosticStatus.ERROR,
                "invalid RGB-D input",
                [("error", error)],
            )
            return

        if self.publish_debug_mask:
            debug_mask = estimate.mask if estimate is not None else np.zeros(
                bgr.shape[:2], dtype=np.uint8
            )
            debug_message = self.bridge.cv2_to_imgmsg(debug_mask, encoding="mono8")
            debug_message.header = color_message.header
            self.debug_pub.publish(debug_message)

        if estimate is None:
            self._publish_empty(color_message)
            return

        source_frame = color_message.header.frame_id
        if not source_frame:
            self.get_logger().error(
                "color image has an empty frame_id", throttle_duration_sec=2.0
            )
            self._publish_metrics(
                color_message,
                DiagnosticStatus.ERROR,
                "empty input frame",
                [],
            )
            return

        source_pose = PoseStamped()
        source_pose.header = color_message.header
        source_pose.pose.position.x = estimate.position[0]
        source_pose.pose.position.y = estimate.position[1]
        source_pose.pose.position.z = estimate.position[2]
        source_pose.pose.orientation.w = 1.0
        output_pose = source_pose
        rotation = np.eye(3)
        output_frame = self.output_frame or source_frame
        if output_frame != source_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    output_frame,
                    source_frame,
                    Time.from_msg(color_message.header.stamp),
                    timeout=Duration(seconds=self.transform_timeout),
                )
                output_pose = do_transform_pose_stamped(source_pose, transform)
                q = transform.transform.rotation
                rotation = rotation_matrix(q.x, q.y, q.z, q.w)
            except TransformException as error:
                self.get_logger().warn(
                    f"target dropped: cannot transform {source_frame} -> "
                    f"{output_frame}: {error}",
                    throttle_duration_sec=2.0,
                )
                self._publish_metrics(
                    color_message,
                    DiagnosticStatus.ERROR,
                    "target TF unavailable",
                    [("source_frame", source_frame), ("output_frame", output_frame)],
                )
                return

        covariance = pose_covariance(estimate, rotation)
        self._publish_detection(
            color_message,
            output_pose,
            estimate,
            covariance,
        )

    def _publish_detection(
        self,
        color_message: Image,
        output_pose: PoseStamped,
        estimate: TargetEstimate,
        covariance: list[float],
    ) -> None:
        pose = PoseWithCovarianceStamped()
        pose.header = output_pose.header
        pose.pose.pose = output_pose.pose
        pose.pose.covariance = covariance
        self.pose_pub.publish(pose)

        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = self.class_id
        result.hypothesis.score = estimate.confidence
        result.pose.pose = output_pose.pose
        result.pose.covariance = covariance

        detection = Detection3D()
        detection.header = output_pose.header
        detection.id = self.target_id
        detection.results.append(result)
        detection.bbox.center = output_pose.pose
        detection.bbox.size.x = estimate.size[0]
        detection.bbox.size.y = estimate.size[1]
        detection.bbox.size.z = estimate.size[2]
        detections = Detection3DArray()
        detections.header = output_pose.header
        detections.detections.append(detection)
        self.detections_3d_pub.publish(detections)

        detection_2d = Detection2D()
        detection_2d.header = color_message.header
        detection_2d.id = self.target_id
        result_2d = ObjectHypothesisWithPose()
        result_2d.hypothesis.class_id = self.class_id
        result_2d.hypothesis.score = estimate.confidence
        result_2d.pose.pose.position.x = estimate.position[0]
        result_2d.pose.pose.position.y = estimate.position[1]
        result_2d.pose.pose.position.z = estimate.position[2]
        result_2d.pose.pose.orientation.w = 1.0
        result_2d.pose.covariance = pose_covariance(estimate, np.eye(3))
        detection_2d.results.append(result_2d)
        _, _, width, height = estimate.bbox_xywh
        detection_2d.bbox.center.position.x = estimate.centroid_uv[0]
        detection_2d.bbox.center.position.y = estimate.centroid_uv[1]
        detection_2d.bbox.size_x = float(width)
        detection_2d.bbox.size_y = float(height)
        detections_2d = Detection2DArray()
        detections_2d.header = color_message.header
        detections_2d.detections.append(detection_2d)
        self.detections_2d_pub.publish(detections_2d)

        self._publish_metrics(
            color_message,
            DiagnosticStatus.OK,
            "target detected",
            [
                ("target_id", self.target_id),
                ("class_id", self.class_id),
                ("output_frame", output_pose.header.frame_id),
                ("valid_depth_points", estimate.valid_depth_points),
                ("valid_depth_ratio", f"{estimate.valid_depth_ratio:.6f}"),
                ("contour_area_px", f"{estimate.contour_area:.3f}"),
                ("confidence", f"{estimate.confidence:.6f}"),
            ],
        )

    def _publish_empty(self, color_message: Image) -> None:
        detections_3d = Detection3DArray()
        detections_3d.header = color_message.header
        self.detections_3d_pub.publish(detections_3d)
        detections_2d = Detection2DArray()
        detections_2d.header = color_message.header
        self.detections_2d_pub.publish(detections_2d)
        self._publish_metrics(
            color_message,
            DiagnosticStatus.WARN,
            "target not detected",
            [("valid_depth_points", 0), ("confidence", "0.0")],
        )

    def _publish_metrics(
        self,
        source: Image,
        level: int,
        message: str,
        values: list[tuple[str, object]],
    ) -> None:
        status = DiagnosticStatus()
        status.level = level
        status.name = "target_perception/color_depth_detector"
        status.hardware_id = "camera_independent_rgbd"
        status.message = message
        status.values = [_key_value(key, value) for key, value in values]
        diagnostics = DiagnosticArray()
        diagnostics.header = source.header
        diagnostics.status.append(status)
        self.metrics_pub.publish(diagnostics)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorDepthDetector()
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
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
