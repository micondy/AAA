#!/usr/bin/env python3
"""Measure ComputePathToPose without commanding robot motion."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def shortest_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def make_pose(node: Node, frame_id: str, values: list[float]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(values[0])
    pose.pose.position.y = float(values[1])
    pose.pose.orientation.z = math.sin(float(values[2]) / 2.0)
    pose.pose.orientation.w = math.cos(float(values[2]) / 2.0)
    return pose


def path_metrics(path, requested_start: list[float], requested_goal: list[float]) -> dict[str, float | int | None]:
    length = 0.0
    yaw_change = 0.0
    pure_rotation_segments = 0
    for previous, current in zip(path.poses, path.poses[1:]):
        dx = current.pose.position.x - previous.pose.position.x
        dy = current.pose.position.y - previous.pose.position.y
        distance = math.hypot(dx, dy)
        delta_yaw = abs(
            shortest_angle(
                yaw_from_quaternion(current.pose.orientation)
                - yaw_from_quaternion(previous.pose.orientation)
            )
        )
        length += distance
        yaw_change += delta_yaw
        if distance < 1e-5 and delta_yaw > 1e-5:
            pure_rotation_segments += 1
    metrics = {
        "path_poses": len(path.poses),
        "translation_length_m": round(length, 6),
        "total_yaw_change_rad": round(yaw_change, 6),
        "pure_rotation_segments": pure_rotation_segments,
    }
    if not path.poses:
        return {
            **metrics,
            "path_start_x": None,
            "path_start_y": None,
            "path_start_yaw": None,
            "path_end_x": None,
            "path_end_y": None,
            "path_end_yaw": None,
            "start_position_error_m": None,
            "goal_position_error_m": None,
            "goal_yaw_error_rad": None,
        }
    first = path.poses[0].pose
    last = path.poses[-1].pose
    first_yaw = yaw_from_quaternion(first.orientation)
    last_yaw = yaw_from_quaternion(last.orientation)
    return {
        **metrics,
        "path_start_x": round(first.position.x, 6),
        "path_start_y": round(first.position.y, 6),
        "path_start_yaw": round(first_yaw, 6),
        "path_end_x": round(last.position.x, 6),
        "path_end_y": round(last.position.y, 6),
        "path_end_yaw": round(last_yaw, 6),
        "start_position_error_m": round(
            math.hypot(first.position.x - requested_start[0], first.position.y - requested_start[1]),
            6,
        ),
        "goal_position_error_m": round(
            math.hypot(last.position.x - requested_goal[0], last.position.y - requested_goal[1]),
            6,
        ),
        "goal_yaw_error_rad": round(abs(shortest_angle(last_yaw - requested_goal[2])), 6),
    }


class PlannerBenchmark(Node):
    def __init__(self) -> None:
        super().__init__("omni_lattice_planner_benchmark")
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def run_case(
        self,
        frame_id: str,
        scenario: str,
        case: dict,
        planner_label: str,
        repeat: int,
        timeout: float,
    ) -> dict:
        goal = ComputePathToPose.Goal()
        goal.start = make_pose(self, frame_id, case["start"])
        goal.goal = make_pose(self, frame_id, case["goal"])
        goal.planner_id = "GridBased"
        goal.use_start = True

        started = time.monotonic()
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        base = {
            "planner": planner_label,
            "scenario": scenario,
            "case": str(case["name"]),
            "repeat": repeat,
        }
        if not send_future.done() or send_future.result() is None:
            return {**base, "status": "SEND_TIMEOUT", "wall_time_ms": round((time.monotonic() - started) * 1000, 3)}
        handle = send_future.result()
        if not handle.accepted:
            return {**base, "status": "REJECTED", "wall_time_ms": round((time.monotonic() - started) * 1000, 3)}

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return {**base, "status": "TIMEOUT", "wall_time_ms": round((time.monotonic() - started) * 1000, 3)}

        wrapped = result_future.result()
        result = wrapped.result
        path = result.path
        metrics = path_metrics(path, case["start"], case["goal"])
        action_success = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and len(path.poses) > 0
            and int(getattr(result, "error_code", 0)) == 0
        )
        position_valid = (
            metrics["start_position_error_m"] is not None
            and metrics["start_position_error_m"] <= 0.11
            and metrics["goal_position_error_m"] <= 0.11
        )
        # SE(2) lattice experiments promise a goal pose. Two-dimensional
        # baselines only promise the goal position and report yaw error as data.
        yaw_valid = "lattice" not in planner_label or metrics["goal_yaw_error_rad"] <= 0.30
        success = action_success and position_valid and yaw_valid
        planning_time = getattr(result, "planning_time", None)
        planning_ms = None
        if planning_time is not None:
            planning_ms = planning_time.sec * 1000.0 + planning_time.nanosec / 1e6
        return {
            **base,
            "status": (
                "SUCCEEDED"
                if success
                else "INVALID_ENDPOINT"
                if action_success
                else "FAILED"
            ),
            "error_code": int(getattr(result, "error_code", 0)),
            "error_msg": str(getattr(result, "error_msg", "")),
            "action_status": int(wrapped.status),
            "wall_time_ms": round((time.monotonic() - started) * 1000, 3),
            "planning_time_ms": None if planning_ms is None else round(planning_ms, 3),
            **metrics,
        }


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    share = Path(get_package_share_directory("omni_lattice_benchmark"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", type=Path, default=share / "config" / "scenarios.yaml")
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--planner-label", required=True)
    parser.add_argument("--repeat", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output-dir", type=Path, default=Path("results/navigation/omni_lattice"))
    return parser.parse_known_args()


def write_records(output: Path, records: list[dict]) -> tuple[Path, Path]:
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = output / f"planner_benchmark_{stamp}.json"
    csv_path = output / f"planner_benchmark_{stamp}.csv"
    json_path.write_text(json.dumps(records, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    fields = sorted({key for record in records for key in record})
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    return json_path, csv_path


def main() -> int:
    args, ros_args = parse_args()
    document = yaml.safe_load(args.scenarios.read_text(encoding="utf-8"))
    if args.scenario not in document["scenarios"]:
        raise ValueError(f"Unknown scenario {args.scenario!r}")
    scenario = document["scenarios"][args.scenario]
    if args.repeat < 1:
        raise ValueError("--repeat must be at least 1")

    rclpy.init(args=ros_args)
    node = PlannerBenchmark()
    records = []
    try:
        if not node.client.wait_for_server(timeout_sec=20.0):
            node.get_logger().error("/compute_path_to_pose is unavailable")
            return 2
        for repeat in range(1, args.repeat + 1):
            for case in scenario["cases"]:
                record = node.run_case(
                    document.get("frame_id", "map"),
                    args.scenario,
                    case,
                    args.planner_label,
                    repeat,
                    args.timeout,
                )
                records.append(record)
                node.get_logger().info(
                    f"{case['name']} #{repeat}: {record['status']} "
                    f"wall={record['wall_time_ms']:.1f} ms"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    json_path, csv_path = write_records(args.output_dir, records)
    succeeded = sum(record["status"] == "SUCCEEDED" for record in records)
    print(f"Completed {succeeded}/{len(records)} plans")
    print(f"JSON: {json_path}")
    print(f"CSV:  {csv_path}")
    return 0 if succeeded == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
