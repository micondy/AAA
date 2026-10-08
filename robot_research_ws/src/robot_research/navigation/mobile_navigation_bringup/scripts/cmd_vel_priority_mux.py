#!/usr/bin/env python3
"""Give fresh manual commands priority over Nav2 and publish one Isaac command stream."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelPriorityMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_priority_mux")
        self.declare_parameter("manual_timeout", 0.30)
        self.manual_timeout = float(self.get_parameter("manual_timeout").value)
        self.manual_deadline = 0.0
        self.output = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_nav_safe", self._on_nav, 10)
        self.create_subscription(Twist, "/cmd_vel_web", self._on_manual, 10)
        self.get_logger().info(
            "Velocity arbitration active: web manual > Nav2 safe output > Isaac /cmd_vel"
        )

    def _on_manual(self, message: Twist) -> None:
        self.manual_deadline = time.monotonic() + self.manual_timeout
        self.output.publish(message)

    def _on_nav(self, message: Twist) -> None:
        if time.monotonic() >= self.manual_deadline:
            self.output.publish(message)


def main() -> None:
    rclpy.init()
    node = CmdVelPriorityMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
