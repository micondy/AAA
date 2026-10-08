#!/usr/bin/env python3
"""Verify an Isaac obstacle through rendered RGB-D and collision_monitor."""

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
from std_srvs.srv import SetBool

OBSTACLE_TYPES = ("low", "hanging", "shelf")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path, default=Path("results/navigation/safety")
    )
    return parser.parse_known_args()


class IsaacObstacleBenchmark(Node):
    def __init__(self) -> None:
        super().__init__("isaac_obstacle_safety_benchmark")
        self.outputs: list[tuple[float, float]] = []
        self.states: list[tuple[float, int, str]] = []
        self.command_pub = self.create_publisher(
            Twist, "/safety_test/cmd_vel_in", 10
        )
        self.obstacle_clients = {
            obstacle_type: self.create_client(
                SetBool, f"/sim/nav_obstacle/{obstacle_type}/set_enabled"
            )
            for obstacle_type in OBSTACLE_TYPES
        }
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

    def set_obstacle(self, obstacle_type: str, enabled: bool) -> bool:
        future = self.obstacle_clients[obstacle_type].call_async(
            SetBool.Request(data=enabled)
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.done() and future.result() and future.result().success)

    def run_phase(self, name: str, duration: float) -> dict:
        started = time.monotonic()
        output_start = len(self.outputs)
        state_start = len(self.states)
        command = Twist()
        command.linear.x = 0.15
        next_command = 0.0
        while time.monotonic() - started < duration:
            now = time.monotonic()
            if now >= next_command:
                self.command_pub.publish(command)
                next_command = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        output = self.outputs[output_start:]
        steady = [speed for stamp, speed in output if stamp >= started + 1.0]
        states = self.states[state_start:]
        return {
            "name": name,
            "output_samples": len(output),
            "steady_samples": len(steady),
            "steady_max_abs_output_mps": max(
                (abs(speed) for speed in steady), default=math.nan
            ),
            "last_output_mps": output[-1][1] if output else math.nan,
            "states": sorted(
                {f"{action}:{polygon}" for _, action, polygon in states}
            ),
            "front_stop_seen": any(
                action == CollisionMonitorState.STOP and polygon == "FrontStop"
                for _, action, polygon in states
            ),
        }


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = IsaacObstacleBenchmark()
    try:
        for obstacle_type, client in node.obstacle_clients.items():
            if not client.wait_for_service(timeout_sec=15.0):
                node.get_logger().error(
                    f"Isaac {obstacle_type} obstacle service is unavailable"
                )
                return 2
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and node.command_pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.command_pub.get_subscription_count() == 0:
            node.get_logger().error("collision_monitor input is unavailable")
            return 2
        for obstacle_type in OBSTACLE_TYPES:
            if not node.set_obstacle(obstacle_type, False):
                return 2
        clear_before = node.run_phase("clear_before", 3.0)
        clear_before["passed"] = clear_before["steady_max_abs_output_mps"] >= 0.12
        records = [clear_before]
        for obstacle_type in OBSTACLE_TYPES:
            if not node.set_obstacle(obstacle_type, True):
                return 2
            occupied = node.run_phase(f"{obstacle_type}_rendered_obstacle", 4.0)
            occupied["obstacle_type"] = obstacle_type
            occupied["passed"] = (
                occupied["steady_max_abs_output_mps"] <= 1.0e-4
                and occupied["front_stop_seen"]
            )
            records.append(occupied)
            if not node.set_obstacle(obstacle_type, False):
                return 2
            # Let depth rays clear the removed object before the next shape.
            node.run_phase(f"{obstacle_type}_clear_transition", 2.0)

        clear_after = node.run_phase("cleared_after_removal", 4.0)
        clear_after["passed"] = clear_after["steady_max_abs_output_mps"] >= 0.12
        records.append(clear_after)
        passed = all(record["passed"] for record in records)
        result = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "passed": passed,
            "pipeline": "Isaac typed obstacles -> RGB-D -> PointCloud2 -> collision_monitor",
            "phases": records,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        output = args.output_dir / (
            "isaac_obstacle_safety_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".json"
        )
        output.write_text(
            json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
        )
        for record in records:
            print(
                f"{record['name']}: passed={record['passed']} "
                f"last={record['last_output_mps']:.4f} states={record['states']}"
            )
        print(f"RESULT: {'PASS' if passed else 'FAIL'}")
        print(f"JSON: {output}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
