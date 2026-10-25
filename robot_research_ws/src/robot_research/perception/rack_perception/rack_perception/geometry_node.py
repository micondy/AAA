"""Extract map-frame layer surfaces and front edges from mask plus aligned depth."""

from __future__ import annotations

from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from rack_perception.geometry import (
    deproject_mask,
    depth_to_meters,
    fit_layer,
    quaternion_matrix,
    transform_points,
)
from rack_perception.model import encode_observations


class RackGeometry(Node):
    def __init__(self) -> None:
        super().__init__("rack_geometry")
        defaults = {
            "mask_topic": "/rack_detector/instance_mask",
            "depth_topic": "/sensors/nav_camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/sensors/nav_camera/color/camera_info",
            "observations_topic": "/rack_geometry/observations",
            "output_frame": "odom",
            "depth_scale_16u": 0.001,
            "min_depth_m": 0.20,
            "max_depth_m": 4.0,
            "min_points": 80,
            "min_confidence": 0.0,
            "grid_height_tolerance_m": 0.08,
            "filter_expected_heights": False,
            "expected_layer_heights_m": [0.55, 1.05, 1.55],
            "bed_length_m": 2.0,
            "top_band_m": 0.04,
            "use_axis_prior": False,
            "axis_prior_yaw_rad": 0.0,
            "sync_queue_size": 15,
            "sync_slop_s": 0.08,
            "transform_timeout_s": 0.12,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.publisher = self.create_publisher(
            String, str(self.get_parameter("observations_topic").value), 10
        )
        mask = Subscriber(self, Image, str(self.get_parameter("mask_topic").value), qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, str(self.get_parameter("depth_topic").value), qos_profile=qos_profile_sensor_data)
        info = Subscriber(self, CameraInfo, str(self.get_parameter("camera_info_topic").value), qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [mask, depth, info],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_s").value),
        )
        self.sync.registerCallback(self._callback)
        self.get_logger().info(f"rack geometry ready: output_frame={self.output_frame}")

    def _callback(self, mask_message: Image, depth_message: Image, info: CameraInfo) -> None:
        try:
            mask = np.asarray(self.bridge.imgmsg_to_cv2(mask_message, "passthrough"))
            depth_raw = np.asarray(self.bridge.imgmsg_to_cv2(depth_message, "passthrough"))
            depth = depth_to_meters(
                depth_raw, depth_message.encoding, float(self.get_parameter("depth_scale_16u").value)
            )
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                depth_message.header.frame_id,
                Time.from_msg(depth_message.header.stamp),
                timeout=Duration(seconds=float(self.get_parameter("transform_timeout_s").value)),
            )
        except (CvBridgeError, TransformException, ValueError) as error:
            self.get_logger().warning(f"rack geometry frame skipped: {error}", throttle_duration_sec=2.0)
            return
        q = transform.transform.rotation
        rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
        t = transform.transform.translation
        translation = np.asarray((t.x, t.y, t.z), dtype=np.float64)
        camera_map = tuple(translation)
        intrinsics = (info.k[0], info.k[4], info.k[2], info.k[5])
        observations = []
        for layer_id in sorted(int(value) for value in np.unique(mask) if 0 < value < 10000):
            optical_points = deproject_mask(
                mask == layer_id,
                depth,
                intrinsics,
                float(self.get_parameter("min_depth_m").value),
                float(self.get_parameter("max_depth_m").value),
            )
            estimate = fit_layer(
                transform_points(optical_points, rotation, translation),
                camera_map,
                layer_id,
                min_points=int(self.get_parameter("min_points").value),
                top_band_m=float(self.get_parameter("top_band_m").value),
                bed_length_m=float(self.get_parameter("bed_length_m").value),
                axis_prior_yaw_rad=(
                    float(self.get_parameter("axis_prior_yaw_rad").value)
                    if bool(self.get_parameter("use_axis_prior").value) else None
                ),
            )
            if estimate is None:
                continue
            if estimate.confidence < float(self.get_parameter("min_confidence").value):
                continue
            # The three known tray planes are a geometric scene prior, not
            # information decoded from the frame-local mask ID.
            if bool(self.get_parameter("filter_expected_heights").value):
                heights = self.get_parameter("expected_layer_heights_m").value
                tolerance = float(self.get_parameter("grid_height_tolerance_m").value)
                if not any(abs(estimate.center[2] - float(height)) <= tolerance for height in heights):
                    continue
            observations.append(estimate)
        output = String()
        stamp_s = depth_message.header.stamp.sec + depth_message.header.stamp.nanosec * 1.0e-9
        output.data = encode_observations(observations, self.output_frame, stamp_s)
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RackGeometry()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
