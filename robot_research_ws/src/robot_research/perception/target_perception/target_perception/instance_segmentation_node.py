"""ROS 2 multi-instance RGB-D observations from torchvision Mask R-CNN."""

from __future__ import annotations

import math
import time

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
    depth_to_meters,
    estimate_mask_target,
)
from target_perception.color_depth_detector_node import (
    pose_covariance,
    rotation_matrix,
)
from target_perception.instance_segmentation import (
    draw_instance_overlay,
    TorchvisionMaskRCNNBackend,
)


def _key_value(key: str, value: object) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = str(value)
    return item


class InstanceSegmentationNode(Node):
    """Fuse general COCO instance masks with aligned depth."""

    def __init__(self) -> None:
        super().__init__("instance_segmentation")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.output_frame = str(self.get_parameter("output_frame").value).strip()
        self.primary_class_id = str(
            self.get_parameter("primary_class_id").value
        ).strip()
        self.depth_scale = float(self.get_parameter("depth_scale_16u").value)
        self.transform_timeout = float(
            self.get_parameter("transform_timeout_s").value
        )
        self.inference_period = 1.0 / float(
            self.get_parameter("inference_rate_hz").value
        )
        self.last_inference_time = -math.inf
        self.max_instances = int(self.get_parameter("max_instances").value)

        self.backend = TorchvisionMaskRCNNBackend(
            weights_path=str(self.get_parameter("weights_path").value),
            device=str(self.get_parameter("device").value),
            score_threshold=float(self.get_parameter("score_threshold").value),
            mask_threshold=float(self.get_parameter("mask_threshold").value),
            allowed_class_names=tuple(
                str(item) for item in self.get_parameter("allowed_classes").value
            ),
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
            str(self.get_parameter("debug_instances_topic").value),
            qos_profile_sensor_data,
        )
        self.overlay_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_overlay_topic").value),
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
            "General instance segmentation ready: "
            f"device={self.backend.device}, weights_sha256="
            f"{self.backend.weights_sha256[:12]}, output_frame="
            f"{self.output_frame or '<input>'}"
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
            "debug_instances_topic": "/target_perception/debug/instances",
            "debug_overlay_topic": "/target_perception/debug/overlay",
            "output_frame": "base_link",
            "weights_path": "",
            "device": "auto",
            "allowed_classes": ["*"],
            "primary_class_id": "",
            "score_threshold": 0.60,
            "mask_threshold": 0.50,
            "max_instances": 10,
            "min_mask_area": 100.0,
            "min_depth_points": 40,
            "min_depth_m": 0.20,
            "max_depth_m": 3.0,
            "depth_scale_16u": 0.001,
            "inference_rate_hz": 2.0,
            "sync_queue_size": 10,
            "sync_slop_s": 0.05,
            "transform_timeout_s": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if float(self.get_parameter("inference_rate_hz").value) <= 0.0:
            raise ValueError("inference_rate_hz must be positive")
        if int(self.get_parameter("max_instances").value) <= 0:
            raise ValueError("max_instances must be positive")

    def _callback(
        self,
        color_message: Image,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        now = time.monotonic()
        if now - self.last_inference_time < self.inference_period:
            return
        self.last_inference_time = now
        started = time.perf_counter()
        try:
            bgr = np.asarray(
                self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            )
            raw_depth = self.bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
            depth_m = depth_to_meters(
                np.asarray(raw_depth), depth_message.encoding, self.depth_scale
            )
            if bgr.shape[:2] != depth_m.shape:
                raise ValueError("color and aligned depth dimensions must match")
            intrinsics = CameraIntrinsics(
                fx=float(camera_info.k[0]),
                fy=float(camera_info.k[4]),
                cx=float(camera_info.k[2]),
                cy=float(camera_info.k[5]),
            )
            predictions = self.backend.infer(bgr)[: self.max_instances]
        except (CvBridgeError, RuntimeError, ValueError) as error:
            self.get_logger().error(
                f"instance inference rejected: {error}", throttle_duration_sec=2.0
            )
            self._publish_metrics(
                color_message,
                DiagnosticStatus.ERROR,
                "instance inference failed",
                [("error", error)],
            )
            return

        source_frame = color_message.header.frame_id
        if not source_frame:
            self._publish_metrics(
                color_message,
                DiagnosticStatus.ERROR,
                "empty input frame",
                [],
            )
            return
        output_frame = self.output_frame or source_frame
        transform = None
        rotation = np.eye(3)
        if output_frame != source_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    output_frame,
                    source_frame,
                    Time.from_msg(color_message.header.stamp),
                    timeout=Duration(seconds=self.transform_timeout),
                )
                q = transform.transform.rotation
                rotation = rotation_matrix(q.x, q.y, q.z, q.w)
            except TransformException as error:
                self._publish_metrics(
                    color_message,
                    DiagnosticStatus.ERROR,
                    "target TF unavailable",
                    [("error", error)],
                )
                return

        detections_3d = Detection3DArray()
        detections_3d.header = color_message.header
        detections_3d.header.frame_id = output_frame
        detections_2d = Detection2DArray()
        detections_2d.header = color_message.header
        instance_image = np.zeros(depth_m.shape, dtype=np.uint16)
        accepted = []

        for rank, prediction in enumerate(predictions, start=1):
            estimate = estimate_mask_target(
                prediction.mask,
                depth_m,
                intrinsics,
                min_mask_area=float(self.get_parameter("min_mask_area").value),
                min_depth_points=int(
                    self.get_parameter("min_depth_points").value
                ),
                min_depth_m=float(self.get_parameter("min_depth_m").value),
                max_depth_m=float(self.get_parameter("max_depth_m").value),
                prior_confidence=prediction.score,
            )
            if estimate is None:
                continue
            target_id = f"{prediction.class_id}_{rank:02d}"
            source_pose = PoseStamped()
            source_pose.header = color_message.header
            source_pose.pose.position.x = estimate.position[0]
            source_pose.pose.position.y = estimate.position[1]
            source_pose.pose.position.z = estimate.position[2]
            source_pose.pose.orientation.w = 1.0
            output_pose = (
                do_transform_pose_stamped(source_pose, transform)
                if transform is not None
                else source_pose
            )
            covariance = pose_covariance(estimate, rotation)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = prediction.class_id
            hypothesis.hypothesis.score = estimate.confidence
            hypothesis.pose.pose = output_pose.pose
            hypothesis.pose.covariance = covariance

            detection_3d = Detection3D()
            detection_3d.header = output_pose.header
            detection_3d.id = target_id
            detection_3d.results.append(hypothesis)
            detection_3d.bbox.center = output_pose.pose
            detection_3d.bbox.size.x = estimate.size[0]
            detection_3d.bbox.size.y = estimate.size[1]
            detection_3d.bbox.size.z = estimate.size[2]
            detections_3d.detections.append(detection_3d)

            detection_2d = Detection2D()
            detection_2d.header = color_message.header
            detection_2d.id = target_id
            detection_2d.results.append(hypothesis)
            x1, y1, x2, y2 = prediction.bbox_xyxy
            detection_2d.bbox.center.position.x = (x1 + x2) / 2.0
            detection_2d.bbox.center.position.y = (y1 + y2) / 2.0
            detection_2d.bbox.size_x = x2 - x1
            detection_2d.bbox.size_y = y2 - y1
            detections_2d.detections.append(detection_2d)
            instance_image[prediction.mask] = rank
            accepted.append((prediction, output_pose, covariance))

        self.detections_3d_pub.publish(detections_3d)
        self.detections_2d_pub.publish(detections_2d)
        debug = self.bridge.cv2_to_imgmsg(instance_image, encoding="16UC1")
        debug.header = color_message.header
        self.debug_pub.publish(debug)
        overlay = self.bridge.cv2_to_imgmsg(
            draw_instance_overlay(bgr, [item[0] for item in accepted]),
            encoding="bgr8",
        )
        overlay.header = color_message.header
        self.overlay_pub.publish(overlay)

        if self.primary_class_id:
            primary = next(
                (
                    item
                    for item in accepted
                    if item[0].class_id == self.primary_class_id
                ),
                None,
            )
            if primary is not None:
                pose = PoseWithCovarianceStamped()
                pose.header = primary[1].header
                pose.pose.pose = primary[1].pose
                pose.pose.covariance = primary[2]
                self.pose_pub.publish(pose)

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        level = DiagnosticStatus.OK if accepted else DiagnosticStatus.WARN
        self._publish_metrics(
            color_message,
            level,
            "instances detected" if accepted else "no depth-valid instances",
            [
                ("model_instances", len(predictions)),
                ("depth_valid_instances", len(accepted)),
                ("inference_and_fusion_ms", f"{elapsed_ms:.3f}"),
                ("device", self.backend.device),
                ("weights_sha256", self.backend.weights_sha256),
                ("primary_class_id", self.primary_class_id or "disabled"),
            ],
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
        status.name = "target_perception/instance_segmentation"
        status.hardware_id = "torchvision_maskrcnn_rgbd"
        status.message = message
        status.values = [_key_value(key, value) for key, value in values]
        diagnostics = DiagnosticArray()
        diagnostics.header = source.header
        diagnostics.status.append(status)
        self.metrics_pub.publish(diagnostics)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = InstanceSegmentationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # ros2 launch can forward another SIGINT while the node is
                # already releasing DDS entities.  Shutdown is still clean.
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
