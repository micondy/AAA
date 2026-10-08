#!/usr/bin/env python3
"""Drive a conservative closed loop for Isaac mapping profiles.

Ground-truth odometry is used only to terminate motion primitives and record
evaluation data.  The comparison odometry topic is configurable so the same
tool can preserve the Carter RGB-D baseline and test the custom base odometry.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Drive and evaluate a low-speed Isaac mapping loop."
    )
    parser.add_argument("--long-edge", type=float, default=1.5)
    parser.add_argument("--short-edge", type=float, default=1.0)
    parser.add_argument("--linear-speed", type=float, default=0.12)
    parser.add_argument("--angular-speed", type=float, default=0.25)
    parser.add_argument(
        "--turn-direction", choices=("left", "right"), default="left"
    )
    parser.add_argument("--settle-seconds", type=float, default=1.5)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--segment-timeout", type=float, default=45.0)
    parser.add_argument("--command-topic", default="/cmd_vel")
    parser.add_argument("--ground-truth-topic", default="/sim/ground_truth/odom")
    parser.add_argument("--estimate-topic", default="/rtabmap/odom")
    parser.add_argument("--output-prefix", type=Path)
    return parser.parse_known_args()


class CarterMappingRoute(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("isaac_mapping_route")
        self.args = args
        self.command_publisher = self.create_publisher(
            Twist, args.command_topic, 10
        )
        self.create_subscription(
            Odometry,
            args.ground_truth_topic,
            self.ground_truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            args.estimate_topic,
            self.rtabmap_callback,
            qos_profile_sensor_data,
        )

        self.ground_truth: Odometry | None = None
        self.rtabmap: Odometry | None = None
        self.previous_ground_truth_yaw: float | None = None
        self.ground_truth_yaw_unwrapped = 0.0
        self.samples: list[dict[str, float]] = []
        self.started_at = time.monotonic()
        self.segment_started_at = self.started_at
        self.settle_until = self.started_at
        self.segment_index = -1
        self.pending_next_segment = False
        self.segment_start: tuple[float, float, float] | None = None
        self.initial_ground_truth: tuple[float, float, float] | None = None
        self.initial_rtabmap: tuple[float, float, float] | None = None
        self.final_ground_truth: tuple[float, float, float] | None = None
        self.final_rtabmap: tuple[float, float, float] | None = None
        self.done = False
        self.failed_reason: str | None = None
        self.last_wait_log = 0.0
        quarter_turn = math.copysign(
            math.pi / 2.0, 1.0 if args.turn_direction == "left" else -1.0
        )
        self.segments: list[tuple[str, float]] = [
            ("drive", args.long_edge),
            ("turn", quarter_turn),
            ("drive", args.short_edge),
            ("turn", quarter_turn),
            ("drive", args.long_edge),
            ("turn", quarter_turn),
            ("drive", args.short_edge),
            ("turn", quarter_turn),
        ]
        self.timer = self.create_timer(0.05, self.control_callback)

    def ground_truth_callback(self, message: Odometry) -> None:
        yaw = yaw_from_odometry(message)
        if self.previous_ground_truth_yaw is None:
            self.ground_truth_yaw_unwrapped = yaw
        else:
            self.ground_truth_yaw_unwrapped += wrap_angle(
                yaw - self.previous_ground_truth_yaw
            )
        self.previous_ground_truth_yaw = yaw
        self.ground_truth = message
        self.record_sample()

    def rtabmap_callback(self, message: Odometry) -> None:
        self.rtabmap = message

    @staticmethod
    def pose(message: Odometry, unwrapped_yaw: float | None = None) -> tuple[float, float, float]:
        position = message.pose.pose.position
        yaw = yaw_from_odometry(message) if unwrapped_yaw is None else unwrapped_yaw
        return position.x, position.y, yaw

    def record_sample(self) -> None:
        if self.ground_truth is None:
            return
        gt_x, gt_y, gt_yaw = self.pose(
            self.ground_truth, self.ground_truth_yaw_unwrapped
        )
        sample = {
            "wall_time_s": time.monotonic() - self.started_at,
            "gt_x_m": gt_x,
            "gt_y_m": gt_y,
            "gt_yaw_rad": gt_yaw,
            "rtab_x_m": math.nan,
            "rtab_y_m": math.nan,
            "rtab_yaw_rad": math.nan,
        }
        if self.rtabmap is not None:
            rtab_x, rtab_y, rtab_yaw = self.pose(self.rtabmap)
            sample.update(
                rtab_x_m=rtab_x,
                rtab_y_m=rtab_y,
                rtab_yaw_rad=rtab_yaw,
            )
        self.samples.append(sample)

    def publish_stop(self) -> None:
        self.command_publisher.publish(Twist())

    def start_next_segment(self) -> None:
        self.publish_stop()
        self.segment_index += 1
        if self.segment_index >= len(self.segments):
            self.final_ground_truth = self.pose(
                self.ground_truth, self.ground_truth_yaw_unwrapped
            )
            self.final_rtabmap = self.pose(self.rtabmap)
            self.done = True
            self.get_logger().info("Closed mapping route completed; robot stopped")
            return
        self.segment_start = self.pose(
            self.ground_truth, self.ground_truth_yaw_unwrapped
        )
        self.segment_started_at = time.monotonic()
        kind, target = self.segments[self.segment_index]
        self.get_logger().info(
            f"Segment {self.segment_index + 1}/{len(self.segments)}: "
            f"{kind} target={target:.3f}"
        )

    def fail(self, reason: str) -> None:
        self.failed_reason = reason
        self.done = True
        self.publish_stop()
        self.get_logger().error(reason)

    def control_callback(self) -> None:
        if self.done:
            self.publish_stop()
            return

        now = time.monotonic()
        if self.ground_truth is None or self.rtabmap is None:
            self.publish_stop()
            if now - self.started_at > self.args.startup_timeout:
                self.fail("Timed out waiting for ground-truth and RTAB-Map odometry")
            elif now - self.last_wait_log > 5.0:
                self.get_logger().info(
                    "Waiting for odometry: "
                    f"truth={self.args.ground_truth_topic}, "
                    f"estimate={self.args.estimate_topic}"
                )
                self.last_wait_log = now
            return

        if self.initial_ground_truth is None:
            self.initial_ground_truth = self.pose(
                self.ground_truth, self.ground_truth_yaw_unwrapped
            )
            self.initial_rtabmap = self.pose(self.rtabmap)
            self.settle_until = now + self.args.settle_seconds
            self.get_logger().info("Both odometry sources ready; settling before motion")
            return

        if now < self.settle_until:
            self.publish_stop()
            return

        if self.pending_next_segment:
            self.pending_next_segment = False
            self.start_next_segment()
            return

        if self.segment_index < 0:
            self.start_next_segment()
            return

        if now - self.segment_started_at > self.args.segment_timeout:
            self.fail(f"Segment {self.segment_index + 1} timed out")
            return

        kind, target = self.segments[self.segment_index]
        start_x, start_y, start_yaw = self.segment_start
        current_x, current_y, current_yaw = self.pose(
            self.ground_truth, self.ground_truth_yaw_unwrapped
        )
        command = Twist()

        if kind == "drive":
            progress = (
                (current_x - start_x) * math.cos(start_yaw)
                + (current_y - start_y) * math.sin(start_yaw)
            )
            remaining = target - progress
            if remaining <= 0.02:
                self.settle_until = now + self.args.settle_seconds
                self.pending_next_segment = True
                self.publish_stop()
                return
            command.linear.x = clamp(
                0.8 * remaining, 0.035, self.args.linear_speed
            )
            heading_error = wrap_angle(start_yaw - current_yaw)
            command.angular.z = clamp(1.2 * heading_error, -0.12, 0.12)
        else:
            progress = current_yaw - start_yaw
            remaining = target - progress
            direction = math.copysign(1.0, target)
            if direction * progress >= abs(target) - math.radians(1.5):
                self.settle_until = now + self.args.settle_seconds
                self.pending_next_segment = True
                self.publish_stop()
                return
            command.angular.z = direction * clamp(
                abs(1.2 * remaining), 0.06, self.args.angular_speed
            )

        self.command_publisher.publish(command)

    @staticmethod
    def path_length(samples: list[dict[str, float]], x_key: str, y_key: str) -> float:
        total = 0.0
        previous: tuple[float, float] | None = None
        for sample in samples:
            point = sample[x_key], sample[y_key]
            if not all(math.isfinite(value) for value in point):
                continue
            if previous is not None:
                total += math.hypot(point[0] - previous[0], point[1] - previous[1])
            previous = point
        return total

    def write_results(self) -> None:
        if self.args.output_prefix is None:
            return
        prefix = self.args.output_prefix.expanduser().resolve()
        prefix.parent.mkdir(parents=True, exist_ok=True)
        csv_path = prefix.with_suffix(".csv")
        json_path = prefix.with_suffix(".json")
        fields = list(self.samples[0]) if self.samples else []
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.samples)

        def closure(initial, final) -> float | None:
            if initial is None or final is None:
                return None
            return math.hypot(final[0] - initial[0], final[1] - initial[1])

        summary = {
            "status": "failed" if self.failed_reason else "completed",
            "failure_reason": self.failed_reason,
            "route": {
                "long_edge_m": self.args.long_edge,
                "short_edge_m": self.args.short_edge,
                "linear_speed_mps": self.args.linear_speed,
                "angular_speed_radps": self.args.angular_speed,
                "turn_direction": self.args.turn_direction,
                "command_topic": self.args.command_topic,
                "ground_truth_topic": self.args.ground_truth_topic,
                "estimate_topic": self.args.estimate_topic,
            },
            "sample_count": len(self.samples),
            "ground_truth_path_length_m": self.path_length(
                self.samples, "gt_x_m", "gt_y_m"
            ),
            "rtabmap_path_length_m": self.path_length(
                self.samples, "rtab_x_m", "rtab_y_m"
            ),
            "ground_truth_closure_translation_m": closure(
                self.initial_ground_truth, self.final_ground_truth
            ),
            "rtabmap_closure_translation_m": closure(
                self.initial_rtabmap, self.final_rtabmap
            ),
        }
        with json_path.open("w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        self.get_logger().info(f"Wrote route results: {json_path}")


def main() -> int:
    args, ros_args = parse_args()
    for name in ("long_edge", "short_edge", "linear_speed", "angular_speed"):
        if getattr(args, name) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    rclpy.init(args=ros_args)
    node = CarterMappingRoute(args)
    interrupted = False

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not node.done and not interrupted:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for _ in range(5):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.03)
        if interrupted and not node.done:
            node.failed_reason = "Interrupted"
        node.write_results()
        failed = node.failed_reason is not None
        node.destroy_node()
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
