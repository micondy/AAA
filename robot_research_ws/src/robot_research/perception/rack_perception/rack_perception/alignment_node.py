"""Continuously publish rack distance, lateral and heading errors."""

from __future__ import annotations

import json
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from rack_perception.alignment import calculate_alignment


class RackAlignment(Node):
    def __init__(self) -> None:
        super().__init__("rack_alignment")
        defaults = {
            "map_topic": "/rack_map/model",
            "errors_topic": "/rack_alignment/errors",
            "diagnostics_topic": "/rack_alignment/status",
            "base_frame": "base_link",
            "target_rack_id": "",
            "target_level": 1,
            "desired_distance_m": 0.65,
            "update_rate_hz": 15.0,
            "transform_timeout_s": 0.08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.model = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, str(self.get_parameter("map_topic").value), self._map_callback, latched)
        self.errors_pub = self.create_publisher(TwistStamped, str(self.get_parameter("errors_topic").value), 10)
        self.status_pub = self.create_publisher(DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10)
        self.create_timer(1.0 / float(self.get_parameter("update_rate_hz").value), self._timer)
        self.get_logger().info("real-time rack alignment ready")

    def _map_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if value.get("schema") == "rack_map/v1":
                self.model = value
        except json.JSONDecodeError:
            self.get_logger().warning("invalid rack map JSON")

    def _target_layer(self):
        target_id = str(self.get_parameter("target_rack_id").value)
        level = int(self.get_parameter("target_level").value)
        for rack in self.model.get("racks", []):
            if target_id and rack["stable_id"] != target_id:
                continue
            for layer in rack.get("layers", []):
                if layer["level_number"] == level:
                    return rack, layer
        return None, None

    def _timer(self) -> None:
        if self.model is None:
            return
        rack, layer = self._target_layer()
        if layer is None:
            return
        frame = str(self.model["frame_id"])
        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
                str(self.get_parameter("base_frame").value),
                Time(),
                timeout=Duration(seconds=float(self.get_parameter("transform_timeout_s").value)),
            )
        except TransformException as error:
            self.get_logger().warning(f"alignment TF unavailable: {error}", throttle_duration_sec=2.0)
            return
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = transform.transform.translation
        result = calculate_alignment(
            (t.x, t.y),
            yaw,
            tuple(layer["front_center"][:2]),
            tuple(layer["front_normal"]),
            tuple(layer["axis"]),
            float(self.get_parameter("desired_distance_m").value),
        )
        message = TwistStamped()
        message.header.frame_id = frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = result.distance_error_m
        message.twist.linear.y = result.lateral_error_m
        message.twist.angular.z = result.angle_error_rad
        self.errors_pub.publish(message)

        status = DiagnosticStatus()
        status.name = "rack_alignment"
        status.hardware_id = rack["stable_id"]
        status.level = DiagnosticStatus.OK
        status.message = "tracking" if layer["observed"] else "using retained rack model"
        status.values = [
            KeyValue(key="rack_id", value=rack["stable_id"]),
            KeyValue(key="layer_id", value=layer["stable_id"]),
            KeyValue(key="current_distance_m", value=f"{result.current_distance_m:.4f}"),
            KeyValue(key="distance_error_m", value=f"{result.distance_error_m:.4f}"),
            KeyValue(key="lateral_error_m", value=f"{result.lateral_error_m:.4f}"),
            KeyValue(key="angle_error_rad", value=f"{result.angle_error_rad:.5f}"),
            KeyValue(key="observed", value=str(bool(layer["observed"])).lower()),
        ]
        array = DiagnosticArray()
        array.header = message.header
        array.status = [status]
        self.status_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RackAlignment()
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
