"""Reproducible runtime validation of the integrated MoveIt collision model."""

from __future__ import annotations

from dataclasses import dataclass
import sys

import rclpy
from moveit_msgs.msg import ContactInformation
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass(frozen=True)
class ValidationCase:
    name: str
    group: str
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    expected_valid: bool
    expected_contact: frozenset[str] | None = None


def arm_joint_names(side: str) -> tuple[str, ...]:
    return tuple(f"openarm_{side}_joint{index}" for index in range(1, 8))


CASES = (
    ValidationCase(
        name="left_home_is_safe",
        group="left_arm",
        joint_names=arm_joint_names("left"),
        positions=(0.0,) * 7,
        expected_valid=True,
    ),
    ValidationCase(
        name="right_home_is_safe",
        group="right_arm",
        joint_names=arm_joint_names("right"),
        positions=(0.0,) * 7,
        expected_valid=True,
    ),
    ValidationCase(
        name="left_link4_body_collision_is_rejected",
        group="",
        joint_names=arm_joint_names("left"),
        positions=(
            1.704292992,
            1.478918891,
            -0.041185155,
            1.476007458,
            0.673356822,
            0.148803935,
            -0.127042762,
        ),
        expected_valid=False,
        expected_contact=frozenset(("openarm_body_link0", "openarm_left_link4")),
    ),
    ValidationCase(
        name="left_finger_work_surface_collision_is_rejected",
        group="",
        joint_names=arm_joint_names("left"),
        positions=(
            -0.677648744,
            0.025895069,
            -0.450875205,
            0.563609678,
            1.430942711,
            0.344878497,
            -0.271307830,
        ),
        expected_valid=False,
        expected_contact=frozenset(("openarm_left_left_finger", "work_surface")),
    ),
)


def contact_bodies(contact: ContactInformation) -> frozenset[str]:
    return frozenset((contact.contact_body_1, contact.contact_body_2))


class CollisionValidator(Node):
    def __init__(self) -> None:
        super().__init__("moveit_collision_validator")
        self._client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )

    def run(self) -> bool:
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/check_state_validity is unavailable")
            return False

        all_passed = True
        for case in CASES:
            request = GetStateValidity.Request()
            request.group_name = case.group
            request.robot_state.is_diff = True
            request.robot_state.joint_state = JointState(
                name=list(case.joint_names), position=list(case.positions)
            )
            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response is None:
                self.get_logger().error(f"FAIL {case.name}: service timeout")
                all_passed = False
                continue

            passed = response.valid == case.expected_valid
            if case.expected_contact is not None:
                passed = passed and any(
                    contact_bodies(contact) == case.expected_contact
                    for contact in response.contacts
                )
            if passed:
                contacts = ", ".join(
                    sorted(" <-> ".join(sorted(contact_bodies(contact)))
                           for contact in response.contacts)
                )
                suffix = f"; contacts={contacts}" if contacts else ""
                self.get_logger().info(
                    f"PASS {case.name}: valid={response.valid}{suffix}"
                )
            else:
                observed = [sorted(contact_bodies(item)) for item in response.contacts]
                self.get_logger().error(
                    f"FAIL {case.name}: valid={response.valid}, contacts={observed}"
                )
                all_passed = False
        return all_passed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionValidator()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if passed else 1)
