#!/usr/bin/env python3
"""Inject deterministic clouds and verify the collision monitor speed boundary."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def parse_args():
    parser = argparse.ArgumentParser(
        description="Verify clear, obstacle-stop and sensor-timeout behavior."
    )
    parser.add_argument("--source-timeout", type=float, default=0.75)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("results/navigation/safety")
    )
    parser.add_argument("--command-speed", type=float, default=0.15)
    return parser.parse_known_args()


class SafetyBenchmark(Node):
    def __init__(self, command_speed: float) -> None:
        super().__init__("collision_safety_benchmark")
        self.command_speed = command_speed
        self.outputs: list[tuple[float, float]] = []
        self.states: list[tuple[float, int, str]] = []
        self.command_pub = self.create_publisher(
            Twist, "/safety_test/cmd_vel_in", 10
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/safety_test/points", qos_profile_sensor_data
        )
        self.create_subscription(
            Twist, "/safety_test/cmd_vel_out", self._on_output, 10
        )
        self.create_subscription(
            CollisionMonitorState, "/safety_test/state", self._on_state, 10
        )

    def _on_output(self, message: Twist) -> None:
        self.outputs.append((time.monotonic(), float(message.linear.x)))

    def _on_state(self, message: CollisionMonitorState) -> None:
        self.states.append(
            (time.monotonic(), int(message.action_type), str(message.polygon_name))
        )

    def make_cloud(self, occupied: bool) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        if occupied:
            points = [(0.50, -0.12 + 0.04 * index, 0.30) for index in range(7)]
        else:
            points = [(1.50, -0.50, 0.30), (1.50, 0.50, 0.30)]
        return point_cloud2.create_cloud_xyz32(header, points)

    def run_phase(self, name: str, duration: float, cloud: str | None) -> dict:
        started = time.monotonic()
        output_start = len(self.outputs)
        state_start = len(self.states)
        command = Twist()
        command.linear.x = self.command_speed
        next_cloud = 0.0
        next_command = 0.0
        while time.monotonic() - started < duration:
            now = time.monotonic()
            if now >= next_command:
                self.command_pub.publish(command)
                next_command = now + 0.05
            if cloud is not None and now >= next_cloud:
                self.cloud_pub.publish(self.make_cloud(cloud == "occupied"))
                next_cloud = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        output_records = self.outputs[output_start:]
        speeds = [speed for _, speed in output_records]
        steady_after = started + min(0.25, duration * 0.5)
        steady_speeds = [
            speed for stamp, speed in output_records if stamp >= steady_after
        ]
        states = self.states[state_start:]
        return {
            "name": name,
            "duration_s": round(time.monotonic() - started, 3),
            "output_samples": len(speeds),
            "max_abs_output_mps": max(
                (abs(value) for value in speeds), default=math.nan
            ),
            "steady_max_abs_output_mps": max(
                (abs(value) for value in steady_speeds), default=math.nan
            ),
            "last_output_mps": speeds[-1] if speeds else math.nan,
            "stop_state_seen": any(
                state[1] == CollisionMonitorState.STOP for state in states
            ),
            "states": sorted(
                {f"{action}:{polygon}" for _, action, polygon in states}
            ),
        }


def evaluate(records: list[dict], command_speed: float) -> None:
    clear, obstacle, timeout = records
    clear["passed"] = (
        clear["output_samples"] > 0
        and clear["steady_max_abs_output_mps"] >= command_speed * 0.8
    )
    obstacle["passed"] = (
        obstacle["output_samples"] > 0
        and obstacle["steady_max_abs_output_mps"] <= 1.0e-4
        and obstacle["stop_state_seen"]
    )
    timeout["passed"] = (
        timeout["output_samples"] > 0
        and abs(timeout["last_output_mps"]) <= 1.0e-4
    )


def main() -> int:
    args, ros_args = parse_args()
    if args.source_timeout <= 0.0:
        raise ValueError("source-timeout must be positive")
    rclpy.init(args=ros_args)
    node = SafetyBenchmark(args.command_speed)
    try:
        deadline = time.monotonic() + 5.0
        while (
            time.monotonic() < deadline
            and node.command_pub.get_subscription_count() == 0
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.command_pub.get_subscription_count() == 0:
            node.get_logger().error("collision_monitor input is unavailable")
            return 2
        records = [
            node.run_phase("clear_cloud_allows_motion", 1.0, "clear"),
            node.run_phase("front_stop_blocks_motion", 1.0, "occupied"),
            node.run_phase("clear_before_timeout", 0.5, "clear"),
            node.run_phase(
                "pointcloud_timeout_blocks_motion", args.source_timeout + 0.6, None
            ),
        ]
        evaluate([records[0], records[1], records[3]], args.command_speed)
        records[2]["passed"] = records[2]["output_samples"] > 0
        passed = all(records[index]["passed"] for index in (0, 1, 3))
        result = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "source_timeout_s": args.source_timeout,
            "command_speed_mps": args.command_speed,
            "passed": passed,
            "phases": records,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = args.output_dir / f"collision_safety_{stamp}.json"
        output.write_text(
            json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
        )
        for record in records:
            print(
                f"{record['name']}: passed={record['passed']} "
                f"samples={record['output_samples']} "
                f"last={record['last_output_mps']:.4f}"
            )
        print(f"RESULT: {'PASS' if passed else 'FAIL'}")
        print(f"JSON: {output}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
