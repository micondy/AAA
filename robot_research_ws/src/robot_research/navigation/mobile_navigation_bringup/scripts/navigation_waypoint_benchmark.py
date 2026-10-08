#!/usr/bin/env python3
"""Run a repeatable sequence of conservative NavigateToPose goals."""

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
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    default_config = (
        Path(get_package_share_directory("mobile_navigation_bringup"))
        / "config"
        / "isaac_indoor_top_rt_01_waypoints.yaml"
    )
    parser = argparse.ArgumentParser(
        description="Preview or execute a fixed Nav2 waypoint benchmark."
    )
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send goals. Without this flag the command is read-only.",
    )
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument(
        "--end-index",
        type=int,
        help="Last waypoint to run (1-based, inclusive; default: all).",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("maps/benchmarks"))
    return parser.parse_known_args()


def load_waypoints(path: Path) -> tuple[str, list[dict[str, float | str]]]:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    frame_id = str(document.get("frame_id", "map"))
    waypoints = document.get("waypoints", [])
    if not waypoints:
        raise ValueError("Expected at least one waypoint")
    required = {"name", "x", "y", "yaw"}
    for index, waypoint in enumerate(waypoints, start=1):
        missing = required - waypoint.keys()
        if missing:
            raise ValueError(f"Waypoint {index} is missing {sorted(missing)}")
    return frame_id, waypoints


class WaypointBenchmark(Node):
    def __init__(self, frame_id: str) -> None:
        super().__init__("navigation_waypoint_benchmark")
        self.frame_id = frame_id
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.feedback: dict[str, float | int] = {}

    def feedback_callback(self, message) -> None:
        feedback = message.feedback
        self.feedback = {
            "distance_remaining_m": float(feedback.distance_remaining),
            "recoveries": int(feedback.number_of_recoveries),
        }

    def run_goal(
        self, waypoint: dict[str, float | str], timeout: float
    ) -> dict[str, float | int | str]:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint["x"])
        goal.pose.pose.position.y = float(waypoint["y"])
        yaw = float(waypoint["yaw"])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        started = time.monotonic()
        self.feedback = {}
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            return self.make_result(waypoint, "SEND_TIMEOUT", started, -1)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return self.make_result(waypoint, "REJECTED", started, -1)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return self.make_result(waypoint, "TIMEOUT", started, -1)

        wrapped = result_future.result()
        status = STATUS_NAMES.get(wrapped.status, str(wrapped.status))
        error_code = int(wrapped.result.error_code)
        return self.make_result(waypoint, status, started, error_code)

    def make_result(self, waypoint, status, started, error_code):
        return {
            "name": str(waypoint["name"]),
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "yaw": float(waypoint["yaw"]),
            "status": status,
            "error_code": error_code,
            "duration_s": round(time.monotonic() - started, 3),
            "distance_remaining_m": self.feedback.get("distance_remaining_m", math.nan),
            "recoveries": self.feedback.get("recoveries", 0),
        }


def write_results(output_dir: Path, records: list[dict]) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = output_dir / f"navigation_waypoints_{stamp}.json"
    csv_path = output_dir / f"navigation_waypoints_{stamp}.csv"
    json_path.write_text(json.dumps(records, indent=2), encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    return json_path, csv_path


def main() -> int:
    args, ros_args = parse_args()
    frame_id, waypoints = load_waypoints(args.config)
    end_index = len(waypoints) if args.end_index is None else args.end_index
    if not 1 <= args.start_index <= end_index <= len(waypoints):
        raise ValueError(
            "Require 1 <= start-index <= end-index <= waypoint count "
            f"({len(waypoints)})"
        )
    selected_waypoints = waypoints[args.start_index - 1 : end_index]
    print(
        f"Selected {len(selected_waypoints)}/{len(waypoints)} waypoints "
        f"in frame '{frame_id}':"
    )
    for index, waypoint in enumerate(selected_waypoints, start=args.start_index):
        print(
            f"  {index:02d} {waypoint['name']}: "
            f"x={float(waypoint['x']):.3f}, y={float(waypoint['y']):.3f}, "
            f"yaw={float(waypoint['yaw']):.3f}"
        )
    if not args.execute:
        print("Dry run only. Add --execute after checking the selected targets in RViz.")
        return 0

    rclpy.init(args=ros_args)
    node = WaypointBenchmark(frame_id)
    records: list[dict] = []
    try:
        if not node.client.wait_for_server(timeout_sec=15.0):
            node.get_logger().error("/navigate_to_pose action server is unavailable")
            return 2
        for index, waypoint in enumerate(selected_waypoints, start=args.start_index):
            node.get_logger().info(
                f"Goal {index}/{len(waypoints)}: {waypoint['name']}"
            )
            record = node.run_goal(waypoint, args.goal_timeout)
            records.append(record)
            node.get_logger().info(
                f"{record['name']}: {record['status']} in {record['duration_s']:.1f}s, "
                f"recoveries={record['recoveries']}"
            )
            if record["status"] != "SUCCEEDED" and not args.continue_on_failure:
                break
            time.sleep(args.settle_seconds)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted; active Nav2 goal will be canceled by shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not records:
        return 2
    json_path, csv_path = write_results(args.output_dir, records)
    succeeded = sum(record["status"] == "SUCCEEDED" for record in records)
    print(f"Completed {succeeded}/{len(records)} goals")
    print(f"JSON: {json_path}")
    print(f"CSV:  {csv_path}")
    return 0 if succeeded == len(selected_waypoints) else 1


if __name__ == "__main__":
    raise SystemExit(main())
