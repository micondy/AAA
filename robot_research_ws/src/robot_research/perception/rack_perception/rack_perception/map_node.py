"""Merge rack observations into stable IDs and retained layer records."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from rack_perception.markers import model_markers
from rack_perception.model import decode_observations
from rack_perception.tracking import RackTracker


class RackMapNode(Node):
    def __init__(self) -> None:
        super().__init__("rack_map")
        defaults = {
            "observations_topic": "/rack_geometry/observations",
            "map_topic": "/rack_map/model",
            "markers_topic": "/rack_map/markers",
            "rack_distance_m": 0.85,
            "group_distance_m": 0.85,
            "layer_xy_distance_m": 0.65,
            "layer_height_distance_m": 0.18,
            "smoothing": 0.25,
            "min_confirmed_layers": 3,
            "min_confirmed_frames": 1,
            "use_source_instance_hints": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.tracker = RackTracker(
            rack_distance_m=float(self.get_parameter("rack_distance_m").value),
            group_distance_m=float(self.get_parameter("group_distance_m").value),
            layer_xy_distance_m=float(self.get_parameter("layer_xy_distance_m").value),
            layer_height_distance_m=float(self.get_parameter("layer_height_distance_m").value),
            smoothing=float(self.get_parameter("smoothing").value),
            min_confirmed_layers=int(self.get_parameter("min_confirmed_layers").value),
            min_confirmed_frames=int(self.get_parameter("min_confirmed_frames").value),
            use_source_instance_hints=bool(self.get_parameter("use_source_instance_hints").value),
        )
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(String, str(self.get_parameter("map_topic").value), latched)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("markers_topic").value), latched)
        self.create_subscription(String, str(self.get_parameter("observations_topic").value), self._callback, 10)
        self.get_logger().info("persistent rack map ready")

    def _callback(self, message: String) -> None:
        try:
            frame_id, stamp_s, observations = decode_observations(message.data)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"invalid rack observations: {error}")
            return
        self.tracker.update(observations, stamp_s)
        model = self.tracker.to_dict(frame_id, stamp_s)
        output = String()
        output.data = json.dumps(model, separators=(",", ":"))
        self.map_pub.publish(output)
        self.marker_pub.publish(model_markers(model, self.get_clock().now().to_msg()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RackMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
