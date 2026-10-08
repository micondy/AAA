from collections import deque
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


def stamp_seconds(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


@dataclass
class StreamState:
    name: str
    minimum_rate: float
    count: int = 0
    previous_count: int = 0
    last_stamp: Optional[float] = None


class CameraHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("camera_health_monitor")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("imu_topic", "/camera/camera/imu")
        self.declare_parameter("report_period", 5.0)
        self.declare_parameter("minimum_image_rate", 20.0)
        self.declare_parameter("minimum_imu_rate", 50.0)
        self.declare_parameter("maximum_rgb_depth_delta_ms", 20.0)
        self.declare_parameter("require_imu", True)

        image_rate = float(self.get_parameter("minimum_image_rate").value)
        imu_rate = float(self.get_parameter("minimum_imu_rate").value)
        self.streams = {
            "color": StreamState("color", image_rate),
            "depth": StreamState("aligned_depth", image_rate),
            "imu": StreamState("imu", imu_rate),
        }
        self.delta_sum_ms = 0.0
        self.delta_max_ms = 0.0
        self.delta_samples = 0
        self.recent_color_stamps = deque(maxlen=10)

        self.create_subscription(
            Image, str(self.get_parameter("color_topic").value),
            self._on_color, qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value),
            self._on_depth, qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value),
            self._on_imu, qos_profile_sensor_data,
        )
        self.create_timer(float(self.get_parameter("report_period").value), self._report)
        self.get_logger().info("Camera health monitoring started")

    def _update(self, key: str, message) -> None:
        state = self.streams[key]
        state.count += 1
        state.last_stamp = stamp_seconds(message)

    def _on_color(self, message: Image) -> None:
        self._update("color", message)
        self.recent_color_stamps.append(stamp_seconds(message))

    def _on_depth(self, message: Image) -> None:
        self._update("depth", message)
        if self.recent_color_stamps:
            depth_stamp = stamp_seconds(message)
            delta_ms = min(
                abs(depth_stamp - color_stamp) for color_stamp in self.recent_color_stamps
            ) * 1000.0
            self.delta_sum_ms += delta_ms
            self.delta_max_ms = max(self.delta_max_ms, delta_ms)
            self.delta_samples += 1

    def _on_imu(self, message: Imu) -> None:
        self._update("imu", message)

    def _report(self) -> None:
        period = float(self.get_parameter("report_period").value)
        rate_parts = []
        unhealthy = False
        for state in self.streams.values():
            rate = (state.count - state.previous_count) / period
            state.previous_count = state.count
            rate_parts.append(f"{state.name}={rate:.1f}Hz")
            required = state.name != "imu" or bool(
                self.get_parameter("require_imu").value
            )
            unhealthy = unhealthy or (required and rate < state.minimum_rate)

        delta_text = "RGB-depth delta=n/a"
        if self.delta_samples:
            mean_ms = self.delta_sum_ms / self.delta_samples
            delta_text = (
                f"RGB-depth delta mean={mean_ms:.2f}ms max={self.delta_max_ms:.2f}ms"
            )
            limit = float(self.get_parameter("maximum_rgb_depth_delta_ms").value)
            unhealthy = unhealthy or self.delta_max_ms > limit

        message = ", ".join(rate_parts) + "; " + delta_text
        if unhealthy:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)
        self.delta_sum_ms = 0.0
        self.delta_max_ms = 0.0
        self.delta_samples = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraHealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
