#!/usr/bin/env python3
"""Move through several rack viewpoints and verify stable three-layer IDs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RackAcceptance(Node):
    def __init__(self, duration_s: float) -> None:
        super().__init__("rack_acceptance")
        self.duration_s = duration_s
        self.wall_start = time.monotonic()
        self.map_frames = 0
        self.stable_frames = 0
        self.invalid_frames = 0
        self.reference_ids = None
        self.id_sets = set()
        self.distances = []
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/rack_map/model", self._map_callback, latched)
        self.create_subscription(TwistStamped, "/rack_alignment/errors", self._error_callback, 10)
        self.create_timer(0.05, self._command_timer)

    def _map_callback(self, message: String) -> None:
        model = json.loads(message.data)
        self.map_frames += 1
        racks = model.get("racks", [])
        if len(racks) != 1 or len(racks[0].get("layers", [])) != 3:
            if self.reference_ids is not None:
                self.invalid_frames += 1
            return
        ids = (racks[0]["stable_id"], *(layer["stable_id"] for layer in racks[0]["layers"]))
        self.id_sets.add(ids)
        if self.reference_ids is None:
            self.reference_ids = ids
        self.stable_frames += 1

    def _error_callback(self, message: TwistStamped) -> None:
        self.distances.append(float(message.twist.linear.x))

    def elapsed(self) -> float:
        return time.monotonic() - self.wall_start

    def _command_timer(self) -> None:
        elapsed = self.elapsed()
        command = Twist()
        if 5.0 <= elapsed < 9.0:
            command.linear.y, command.angular.z = 0.08, 0.04
        elif 9.0 <= elapsed < 13.0:
            command.linear.y, command.angular.z = -0.16, -0.08
        elif 13.0 <= elapsed < 17.0:
            command.linear.y, command.angular.z = 0.08, 0.04
        elif 17.0 <= elapsed < 19.0:
            command.linear.x = 0.08
        elif 19.0 <= elapsed < 21.0:
            command.linear.x = -0.08
        self.publisher.publish(command)

    def result(self) -> dict:
        distance_span = max(self.distances) - min(self.distances) if self.distances else 0.0
        passed = (
            self.stable_frames >= 20
            and self.invalid_frames == 0
            and len(self.id_sets) == 1
            and len(self.distances) >= 20
            and distance_span >= 0.05
        )
        return {
            "passed": passed,
            "duration_s": self.elapsed(),
            "map_frames": self.map_frames,
            "stable_three_layer_frames": self.stable_frames,
            "invalid_frames_after_lock": self.invalid_frames,
            "stable_id_sets": [list(item) for item in sorted(self.id_sets)],
            "alignment_samples": len(self.distances),
            "distance_error_span_m": distance_span,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=23.0)
    parser.add_argument("--output", default="results/rack/acceptance.json")
    arguments = parser.parse_args()
    rclpy.init()
    node = RackAcceptance(arguments.duration)
    try:
        while rclpy.ok() and node.elapsed() < arguments.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.publisher.publish(Twist())
        result = node.result()
        node.destroy_node()
        rclpy.shutdown()
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
