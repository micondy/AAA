#!/usr/bin/env python3
"""Convert one 2-D LaserScan to a PointCloud2 in its original sensor frame."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class LaserScanToCloud(Node):
    def __init__(self):
        super().__init__('laser_scan_to_cloud')
        self.declare_parameter('scan_topic', '/sensors/front_lidar/scan')
        self.declare_parameter('cloud_topic', '/sensors/front_lidar/cloud')
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter('cloud_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.convert, qos_profile_sensor_data)

    def convert(self, scan):
        points = []
        for index, distance in enumerate(scan.ranges):
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                angle = scan.angle_min + index * scan.angle_increment
                points.append((distance * math.cos(angle),
                               distance * math.sin(angle), 0.0))
        if points:
            self.publisher.publish(point_cloud2.create_cloud_xyz32(scan.header, points))


def main():
    rclpy.init()
    node = LaserScanToCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
