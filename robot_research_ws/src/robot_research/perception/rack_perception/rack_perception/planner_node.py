"""Publish a Nav2-compatible parking goal in front of a selected rack layer."""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class HarvestPlanner(Node):
    def __init__(self) -> None:
        super().__init__("harvest_planner")
        defaults = {
            "map_topic": "/rack_map/model",
            "goal_topic": "/harvest_planner/parking_pose",
            "target_rack_id": "",
            "target_level": 1,
            "standoff_m": 0.65,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), latched)
        self.create_subscription(String, str(self.get_parameter("map_topic").value), self._callback, latched)
        self.get_logger().info("rack parking-pose planner ready")

    def _callback(self, message: String) -> None:
        try:
            model = json.loads(message.data)
            racks = model.get("racks", [])
            target_id = str(self.get_parameter("target_rack_id").value)
            rack = next((item for item in racks if not target_id or item["stable_id"] == target_id), None)
            level = int(self.get_parameter("target_level").value)
            layer = next((item for item in rack["layers"] if item["level_number"] == level), None) if rack else None
            if layer is None:
                return
            front = layer["front_center"]
            normal = layer["front_normal"]
            distance = float(self.get_parameter("standoff_m").value)
            yaw = math.atan2(-normal[1], -normal[0])
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"cannot select rack parking pose: {error}")
            return
        goal = PoseStamped()
        goal.header.frame_id = str(model["frame_id"])
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(front[0] + distance * normal[0])
        goal.pose.position.y = float(front[1] + distance * normal[1])
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HarvestPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
