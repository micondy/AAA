"""Expose stable base topics and the single odom -> base_link TF owner."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def twist_is_finite(message: Twist) -> bool:
    values = (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )
    return all(math.isfinite(value) for value in values)


def odometry_to_transform(message: Odometry) -> TransformStamped:
    transform = TransformStamped()
    transform.header = message.header
    transform.child_frame_id = message.child_frame_id
    transform.transform.translation.x = message.pose.pose.position.x
    transform.transform.translation.y = message.pose.pose.position.y
    transform.transform.translation.z = message.pose.pose.position.z
    transform.transform.rotation = message.pose.pose.orientation
    return transform


class BaseIoAdapter(Node):
    """Bridge project-wide names to the existing swerve controller names."""

    def __init__(self) -> None:
        super().__init__("base_io_adapter")
        self.declare_parameter("command_input", "/cmd_vel")
        self.declare_parameter("controller_command", "/swerve_controller/cmd_vel")
        self.declare_parameter("controller_odometry", "/swerve_controller/odom")
        self.declare_parameter("odometry_output", "/odom")
        self.declare_parameter("ground_truth_output", "/sim/ground_truth/odom")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("publish_ground_truth", True)

        command_input = self.get_parameter("command_input").value
        controller_command = self.get_parameter("controller_command").value
        controller_odometry = self.get_parameter("controller_odometry").value
        odometry_output = self.get_parameter("odometry_output").value
        ground_truth_output = self.get_parameter("ground_truth_output").value

        self._command_publisher = self.create_publisher(Twist, controller_command, 10)
        self._odometry_publisher = self.create_publisher(Odometry, odometry_output, 10)
        self._ground_truth_publisher = None
        if self.get_parameter("publish_ground_truth").value:
            self._ground_truth_publisher = self.create_publisher(
                Odometry, ground_truth_output, 10
            )
        self._tf_broadcaster = None
        if self.get_parameter("publish_odom_tf").value:
            self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, command_input, self._on_command, 10)
        self.create_subscription(
            Odometry, controller_odometry, self._on_odometry, 10
        )
        self.get_logger().info(
            f"Base interfaces ready: {command_input} -> {controller_command}, "
            f"{controller_odometry} -> {odometry_output}"
        )

    def _on_command(self, message: Twist) -> None:
        if not twist_is_finite(message):
            self.get_logger().error("Rejected non-finite /cmd_vel")
            return
        self._command_publisher.publish(message)

    def _on_odometry(self, message: Odometry) -> None:
        self._odometry_publisher.publish(message)
        if self._ground_truth_publisher is not None:
            self._ground_truth_publisher.publish(message)
        if self._tf_broadcaster is not None:
            self._tf_broadcaster.sendTransform(odometry_to_transform(message))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseIoAdapter()
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
