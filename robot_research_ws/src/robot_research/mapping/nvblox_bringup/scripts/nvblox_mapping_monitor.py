#!/usr/bin/env python3
"""Record a compact, repeatable health report for an nvblox mapping run."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from nav_msgs.msg import Odometry
from nvblox_msgs.msg import DistanceMapSlice, Mesh
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Measure nvblox mapping health.")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-path-length", type=float, default=0.0)
    parser.add_argument("--maximum-depth-gap", type=float, default=1.0)
    parser.add_argument(
        "--depth-topic",
        default="/sensors/nav_camera/aligned_depth_to_color/image_raw",
    )
    parser.add_argument("--odom-topic", default="/swerve_controller/odom")
    return parser.parse_known_args()


class StreamCounter:
    def __init__(self) -> None:
        self.count = 0
        self.first_wall: float | None = None
        self.last_wall: float | None = None
        self.max_gap_s = 0.0

    def add(self, now: float) -> None:
        if self.last_wall is not None:
            self.max_gap_s = max(self.max_gap_s, now - self.last_wall)
        self.count += 1
        self.first_wall = now if self.first_wall is None else self.first_wall
        self.last_wall = now

    def rate_hz(self) -> float:
        if self.count < 2 or self.first_wall is None or self.last_wall is None:
            return 0.0
        elapsed = self.last_wall - self.first_wall
        return (self.count - 1) / elapsed if elapsed > 0.0 else 0.0


class NvbloxMappingMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("nvblox_mapping_monitor")
        self.started = time.monotonic()
        self.streams = {
            "depth": StreamCounter(),
            "mesh": StreamCounter(),
            "esdf_pointcloud": StreamCounter(),
            "map_slice": StreamCounter(),
            "odom": StreamCounter(),
        }
        self.nonempty_mesh_messages = 0
        self.max_mesh_blocks = 0
        self.max_mesh_vertices = 0
        self.nonempty_esdf_clouds = 0
        self.max_esdf_points = 0
        self.nonempty_slices = 0
        self.max_slice_cells = 0
        self.max_observed_slice_cells = 0
        self.path_length_m = 0.0
        self.previous_odom: tuple[float, float] | None = None
        self.gpu_samples: list[dict[str, float]] = []
        self.next_gpu_sample = self.started
        self.create_subscription(
            Image, args.depth_topic, self.depth_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Mesh, "/nvblox_node/mesh", self.mesh_callback, 10
        )
        self.create_subscription(
            PointCloud2,
            "/nvblox_node/static_esdf_pointcloud",
            self.esdf_callback,
            10,
        )
        self.create_subscription(
            DistanceMapSlice,
            "/nvblox_node/static_map_slice",
            self.slice_callback,
            10,
        )
        self.create_subscription(
            Odometry, args.odom_topic, self.odom_callback, qos_profile_sensor_data
        )

    def sample_gpu(self) -> None:
        now = time.monotonic()
        if now < self.next_gpu_sample:
            return
        self.next_gpu_sample = now + 1.0
        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=memory.used,utilization.gpu,temperature.gpu",
                    "--format=csv,noheader,nounits",
                    "--id=0",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            values = [float(value.strip()) for value in result.stdout.split(",")]
            if result.returncode == 0 and len(values) == 3:
                self.gpu_samples.append(
                    {
                        "memory_used_mb": values[0],
                        "utilization_percent": values[1],
                        "temperature_c": values[2],
                    }
                )
        except (OSError, subprocess.SubprocessError, ValueError):
            return

    def _mark(self, stream: str) -> None:
        self.streams[stream].add(time.monotonic())

    def depth_callback(self, _: Image) -> None:
        self._mark("depth")

    def mesh_callback(self, message: Mesh) -> None:
        self._mark("mesh")
        blocks = len(message.blocks)
        vertices = sum(len(block.vertices) for block in message.blocks)
        self.max_mesh_blocks = max(self.max_mesh_blocks, blocks)
        self.max_mesh_vertices = max(self.max_mesh_vertices, vertices)
        if blocks > 0 and vertices > 0:
            self.nonempty_mesh_messages += 1

    def esdf_callback(self, message: PointCloud2) -> None:
        self._mark("esdf_pointcloud")
        points = int(message.width) * int(message.height)
        self.max_esdf_points = max(self.max_esdf_points, points)
        if points > 0:
            self.nonempty_esdf_clouds += 1

    def slice_callback(self, message: DistanceMapSlice) -> None:
        self._mark("map_slice")
        cells = int(message.width) * int(message.height)
        observed = sum(
            1
            for value in message.data
            if math.isfinite(value) and value != message.unknown_value
        )
        self.max_slice_cells = max(self.max_slice_cells, cells)
        self.max_observed_slice_cells = max(self.max_observed_slice_cells, observed)
        if cells > 0 and observed > 0:
            self.nonempty_slices += 1

    def odom_callback(self, message: Odometry) -> None:
        self._mark("odom")
        position = message.pose.pose.position
        point = float(position.x), float(position.y)
        if self.previous_odom is not None:
            step = math.hypot(
                point[0] - self.previous_odom[0],
                point[1] - self.previous_odom[1],
            )
            if step < 0.25:
                self.path_length_m += step
        self.previous_odom = point

    def report(self, args: argparse.Namespace) -> dict:
        elapsed = time.monotonic() - self.started
        stream_report = {
            name: {
                "count": state.count,
                "rate_hz": state.rate_hz(),
                "max_gap_s": state.max_gap_s,
            }
            for name, state in self.streams.items()
        }
        gpu_report = {"sample_count": len(self.gpu_samples)}
        for key in ("memory_used_mb", "utilization_percent", "temperature_c"):
            values = [sample[key] for sample in self.gpu_samples]
            gpu_report[key] = {
                "mean": statistics.fmean(values) if values else None,
                "max": max(values) if values else None,
            }
        checks = {
            "depth_received": self.streams["depth"].count > 0,
            "mesh_nonempty": self.nonempty_mesh_messages > 0,
            "esdf_cloud_nonempty": self.nonempty_esdf_clouds > 0,
            "esdf_slice_observed": self.nonempty_slices > 0,
            "minimum_path_length": self.path_length_m >= args.minimum_path_length,
            "depth_gap_bounded": (
                self.streams["depth"].count > 1
                and self.streams["depth"].max_gap_s <= args.maximum_depth_gap
            ),
        }
        return {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "duration_wall_s": elapsed,
            "minimum_path_length_m": args.minimum_path_length,
            "maximum_depth_gap_s": args.maximum_depth_gap,
            "path_length_m": self.path_length_m,
            "streams": stream_report,
            "gpu": gpu_report,
            "map_content": {
                "nonempty_mesh_messages": self.nonempty_mesh_messages,
                "max_mesh_blocks": self.max_mesh_blocks,
                "max_mesh_vertices": self.max_mesh_vertices,
                "nonempty_esdf_clouds": self.nonempty_esdf_clouds,
                "max_esdf_points": self.max_esdf_points,
                "nonempty_slices": self.nonempty_slices,
                "max_slice_cells": self.max_slice_cells,
                "max_observed_slice_cells": self.max_observed_slice_cells,
            },
            "checks": checks,
        }


def main() -> int:
    args, ros_args = parse_args()
    if (
        args.duration <= 0.0
        or args.minimum_path_length < 0.0
        or args.maximum_depth_gap <= 0.0
    ):
        raise SystemExit(
            "duration/maximum depth gap must be positive and minimum path length "
            "non-negative"
        )
    rclpy.init(args=ros_args)
    node = NvbloxMappingMonitor(args)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.sample_gpu()
        report = node.report(args)
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        node.get_logger().info(
            f"Nvblox mapping benchmark {report['status']}: {output}"
        )
        return 0 if report["status"] == "PASS" else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
