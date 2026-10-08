#!/usr/bin/env python3
"""Measure typed Isaac obstacles in the nvblox-backed local costmap."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


OBSTACLE_TYPES = ("low", "hanging", "shelf")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path("results/nvblox/nav2_ab"))
    parser.add_argument("--warmup", type=float, default=8.0)
    parser.add_argument("--phase-seconds", type=float, default=4.0)
    parser.add_argument("--clear-seconds", type=float, default=8.0)
    parser.add_argument("--minimum-added-cells", type=int, default=4)
    return parser.parse_known_args()


class CostmapObstacleBenchmark(Node):
    def __init__(self) -> None:
        super().__init__("nvblox_costmap_obstacle_benchmark")
        self.costmap: OccupancyGrid | None = None
        self.odom: Odometry | None = None
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_costmap, 10
        )
        self.create_subscription(
            Odometry, "/swerve_controller/odom", self._on_odom, 20
        )
        self.obstacle_clients = {
            obstacle_type: self.create_client(
                SetBool, f"/sim/nav_obstacle/{obstacle_type}/set_enabled"
            )
            for obstacle_type in OBSTACLE_TYPES
        }

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.costmap = message

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def set_obstacle(self, obstacle_type: str, enabled: bool) -> bool:
        future = self.obstacle_clients[obstacle_type].call_async(
            SetBool.Request(data=enabled)
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.done() and future.result() and future.result().success)

    def lethal_cells_in_front(self) -> int:
        if self.costmap is None or self.odom is None:
            return 0
        grid = self.costmap
        pose = self.odom.pose.pose
        robot_yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        origin = grid.info.origin
        origin_yaw = 2.0 * math.atan2(origin.orientation.z, origin.orientation.w)
        resolution = float(grid.info.resolution)
        cosine_origin, sine_origin = math.cos(origin_yaw), math.sin(origin_yaw)
        cosine_robot, sine_robot = math.cos(robot_yaw), math.sin(robot_yaw)
        count = 0
        for index, cost in enumerate(grid.data):
            if cost < 99:
                continue
            column = index % grid.info.width
            row = index // grid.info.width
            local_x = (column + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            world_x = origin.position.x + cosine_origin * local_x - sine_origin * local_y
            world_y = origin.position.y + sine_origin * local_x + cosine_origin * local_y
            delta_x = world_x - pose.position.x
            delta_y = world_y - pose.position.y
            base_x = cosine_robot * delta_x + sine_robot * delta_y
            base_y = -sine_robot * delta_x + cosine_robot * delta_y
            if 0.30 <= base_x <= 1.20 and abs(base_y) <= 0.55:
                count += 1
        return count

    def sample(self, duration: float) -> dict:
        values = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.costmap is not None and self.odom is not None:
                values.append(self.lethal_cells_in_front())
        return {
            "samples": len(values),
            "median_lethal_cells": statistics.median(values) if values else 0,
            "max_lethal_cells": max(values, default=0),
            "last_lethal_cells": values[-1] if values else 0,
        }


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = CostmapObstacleBenchmark()
    try:
        for obstacle_type, client in node.obstacle_clients.items():
            if not client.wait_for_service(timeout_sec=15.0):
                node.get_logger().error(f"missing {obstacle_type} obstacle service")
                return 2
            if not node.set_obstacle(obstacle_type, False):
                return 2
        baseline = node.sample(args.warmup)
        records = []
        for obstacle_type in OBSTACLE_TYPES:
            if not node.set_obstacle(obstacle_type, True):
                return 2
            occupied = node.sample(args.phase_seconds)
            if not node.set_obstacle(obstacle_type, False):
                return 2
            cleared = node.sample(args.clear_seconds)
            added = occupied["max_lethal_cells"] - baseline["median_lethal_cells"]
            removal_residual = (
                cleared["last_lethal_cells"] - baseline["median_lethal_cells"]
            )
            record = {
                "obstacle_type": obstacle_type,
                "occupied": occupied,
                "cleared": cleared,
                "added_lethal_cells": added,
                "removal_residual_cells": removal_residual,
                "mark_passed": added >= args.minimum_added_cells,
                "clear_passed": removal_residual <= args.minimum_added_cells,
            }
            record["passed"] = record["mark_passed"] and record["clear_passed"]
            records.append(record)
        passed = baseline["samples"] > 0 and all(r["passed"] for r in records)
        result = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "profile": "nvblox_esdf_local_costmap",
            "passed": passed,
            "roi_base_m": {"x_min": 0.30, "x_max": 1.20, "abs_y_max": 0.55},
            "baseline": baseline,
            "obstacles": records,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        output = args.output_dir / (
            "nvblox_costmap_obstacles_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".json"
        )
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        for record in records:
            print(
                f"{record['obstacle_type']}: mark={record['mark_passed']} "
                f"clear={record['clear_passed']} added={record['added_lethal_cells']} "
                f"residual={record['removal_residual_cells']}"
            )
        print(f"RESULT: {'PASS' if passed else 'FAIL'}")
        print(f"JSON: {output}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
