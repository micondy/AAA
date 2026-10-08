"""Publish YDS2MD-1 serial data as ROS 2 /scan without building a package.

Run: source /opt/ros/jazzy/setup.bash
     python3 read_lidar_ros2.py --port /dev/ttyUSB0
"""

import argparse
import math
import os
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from read_lidar_wifi import PacketStream


SAMPLE_COUNT = 720
RANGE_MIN = 0.12
RANGE_MAX = 8.0


class LidarPublisher(Node):
    def __init__(self, publish_preview_tf=True):
        super().__init__('yds2md_serial_reader')
        self.publisher = self.create_publisher(
            LaserScan, '/scan',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        if publish_preview_tf:
            self.static_tf = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = 'laser_frame'
            transform.transform.rotation.w = 1.0
            self.static_tf.sendTransform(transform)
        self.decoder = PacketStream()
        self.points = []
        self.scan_started = False
        self.last_boundary = None
        self.scans_published = 0

    def handle_bytes(self, data):
        for packet_type, points in self.decoder.feed_packets(data):
            if packet_type & 1:
                now = time.monotonic()
                if self.scan_started and len(self.points) >= 50:
                    self.publish_scan(now - self.last_boundary)
                self.points = []
                self.scan_started = True
                self.last_boundary = now
            if self.scan_started:
                self.points.extend(points)

    def publish_scan(self, scan_time):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / SAMPLE_COUNT
        msg.angle_max = math.pi - msg.angle_increment
        msg.scan_time = max(scan_time, 0.0)
        msg.time_increment = 0.0
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        ranges = [math.inf] * SAMPLE_COUNT
        valid_count = 0
        for angle_deg, distance_m in self.points:
            if not RANGE_MIN <= distance_m <= RANGE_MAX:
                continue
            # The device angles increase clockwise; ROS LaserScan increases CCW.
            angle_rad = math.radians(-angle_deg)
            index = round((angle_rad + math.pi) / msg.angle_increment) % SAMPLE_COUNT
            if distance_m < ranges[index]:
                ranges[index] = distance_m
                valid_count += 1
        msg.ranges = ranges
        self.publisher.publish(msg)
        self.scans_published += 1
        if self.scans_published % 10 == 0:
            self.get_logger().info(
                f'published /scan #{self.scans_published}: '
                f'{valid_count} valid bins, scan_time={msg.scan_time:.3f}s'
            )


def main():
    parser = argparse.ArgumentParser(description='Publish wired YDS2MD-1 data to ROS 2 /scan')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    parser.add_argument('--no-preview-tf', action='store_true',
                        help='Do not publish the identity base_link -> laser_frame transform')
    args = parser.parse_args()

    fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    old_settings = termios.tcgetattr(fd)
    rclpy.init()
    node = LidarPublisher(publish_preview_tf=not args.no_preview_tf)
    try:
        tty.setraw(fd, termios.TCSANOW)
        settings = termios.tcgetattr(fd)
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        node.get_logger().info(f'reading {args.port} at 115200 8N1; publishing /scan')
        while rclpy.ok():
            ready, _, _ = select.select([fd], [], [], 0.1)
            if ready:
                try:
                    data = os.read(fd, 8192)
                except BlockingIOError:
                    data = b''
                if data:
                    node.handle_bytes(data)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        node.get_logger().error(f'serial connection lost: {exc}; reconnect and restart this script')
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSANOW, old_settings)
        except termios.error:
            pass
        os.close(fd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
