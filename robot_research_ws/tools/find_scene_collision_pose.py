#!/usr/bin/env python3
"""Find a reproducible OpenArm joint pose that contacts a scene object.

This is a development helper, not part of the normal robot launch path.  It
samples only within the official OpenArm v1.0 position limits and reports the
first whole-robot collision involving one of the requested scene object IDs.
"""

from __future__ import annotations

import argparse
import random
import sys

import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_LIMITS = (
    (-1.396263, 3.490659),
    (-1.745329, 1.745329),
    (-1.570796, 1.570796),
    (0.0, 2.443461),
    (-1.570796, 1.570796),
    (-0.785398, 0.785398),
    (-1.570796, 1.570796),
)


def joint_names(side: str) -> list[str]:
    return [f"openarm_{side}_joint{index}" for index in range(1, 8)]


class SceneCollisionSearch(Node):
    def __init__(self) -> None:
        super().__init__("scene_collision_pose_search")
        self.client = self.create_client(GetStateValidity, "/check_state_validity")

    def check(self, side: str, positions: list[float]):
        request = GetStateValidity.Request()
        request.group_name = ""
        request.robot_state.is_diff = True
        request.robot_state.joint_state = JointState(
            name=joint_names(side), position=positions
        )
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=20260911)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--objects",
        nargs="+",
        default=("work_surface", "shelf_back", "shelf_upper", "occluder"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)
    targets = set(args.objects)
    rclpy.init()
    node = SceneCollisionSearch()
    try:
        if not node.client.wait_for_service(timeout_sec=10.0):
            print("/check_state_validity is unavailable", file=sys.stderr)
            return 2
        for sample_index in range(args.samples):
            positions = [rng.uniform(lower, upper) for lower, upper in JOINT_LIMITS]
            response = node.check(args.side, positions)
            if response is None:
                print(f"service timeout at sample {sample_index}", file=sys.stderr)
                return 2
            pairs = [
                frozenset((contact.contact_body_1, contact.contact_body_2))
                for contact in response.contacts
            ]
            scene_pairs = [pair for pair in pairs if pair & targets]
            if scene_pairs:
                print(f"sample={sample_index} seed={args.seed} side={args.side}")
                print("positions=(" + ", ".join(f"{item:.9f}" for item in positions) + ")")
                print("contacts=" + repr([sorted(pair) for pair in scene_pairs]))
                return 0
        print(f"no scene collision in {args.samples} samples", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
