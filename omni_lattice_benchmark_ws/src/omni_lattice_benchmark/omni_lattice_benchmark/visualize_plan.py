#!/usr/bin/env python3
"""Render a ComputePathToPose result over the benchmark map."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
import yaml

from omni_lattice_benchmark.planner_benchmark import make_pose, yaw_from_quaternion


def load_map(yaml_path: Path) -> tuple[np.ndarray, float, tuple[float, float, float]]:
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    resolution = float(document["resolution"])
    origin = tuple(float(v) for v in document["origin"])
    with Image.open(yaml_path.parent / document["image"]) as image:
        grid = np.asarray(image.convert("L"), dtype=np.float64)
    return grid, resolution, origin


def render(grid, resolution, origin, path, start, goal, planner_label, scenario, case_name, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    height, width = grid.shape
    extent = [
        origin[0],
        origin[0] + width * resolution,
        origin[1],
        origin[1] + height * resolution,
    ]

    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    # FREE=254 (near white), OCCUPIED=0 (black). Show obstacles dark.
    ax.imshow(grid, cmap="gray", extent=extent, origin="upper", vmin=0, vmax=255, zorder=0)

    xs = [pose.pose.position.x for pose in path.poses]
    ys = [pose.pose.position.y for pose in path.poses]
    ax.plot(xs, ys, color="#d62728", linewidth=1.6, zorder=3, label="path")

    # Heading arrows every N poses.
    step = max(1, len(path.poses) // 40)
    for pose in path.poses[::step]:
        yaw = yaw_from_quaternion(pose.pose.orientation)
        ax.annotate(
            "",
            xy=(pose.pose.position.x + 0.16 * math.cos(yaw), pose.pose.position.y + 0.16 * math.sin(yaw)),
            xytext=(pose.pose.position.x, pose.pose.position.y),
            arrowprops={"arrowstyle": "->", "color": "#d62728", "lw": 1.0, "alpha": 0.7},
            zorder=4,
        )

    def draw_pose(position, yaw, color, label):
        ax.scatter([position[0]], [position[1]], color=color, s=90, zorder=5, label=label)
        ax.annotate(
            "",
            xy=(position[0] + 0.4 * math.cos(yaw), position[1] + 0.4 * math.sin(yaw)),
            xytext=(position[0], position[1]),
            arrowprops={"arrowstyle": "->", "color": color, "lw": 2.2},
            zorder=5,
        )

    draw_pose(start, start[2], "#2ca02c", "start")
    draw_pose(goal, goal[2], "#1f77b4", "goal")

    if path.poses:
        last = path.poses[-1].pose
        end_yaw = yaw_from_quaternion(last.orientation)
        pos_err = math.hypot(last.position.x - goal[0], last.position.y - goal[1])
        yaw_err = abs((end_yaw - goal[2] + math.pi) % (2 * math.pi) - math.pi)
        title = (
            f"{planner_label} / {scenario} / {case_name}\n"
            f"end_pos_err={pos_err:.3f} m  end_yaw_err={math.degrees(yaw_err):.1f} deg"
        )
    else:
        title = f"{planner_label} / {scenario} / {case_name}\n(no path)"

    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="lower left", fontsize=9)
    ax.set_xlim(origin[0], origin[0] + width * resolution)
    ax.set_ylim(origin[1], origin[1] + height * resolution)
    fig.tight_layout()
    fig.savefig(output, dpi=110)
    plt.close(fig)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--start", type=float, nargs=3, required=True)
    parser.add_argument("--goal", type=float, nargs=3, required=True)
    parser.add_argument("--planner-label", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args, ros_args = parser.parse_known_args()

    grid, resolution, origin = load_map(args.map_yaml)

    rclpy.init(args=ros_args)
    node = Node("omni_lattice_plan_visualizer")
    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    try:
        if not client.wait_for_server(timeout_sec=20.0):
            node.get_logger().error("/compute_path_to_pose is unavailable")
            return 2
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = make_pose(node, "map", args.start)
        goal_msg.goal = make_pose(node, "map", args.goal)
        goal_msg.planner_id = "GridBased"
        goal_msg.use_start = True

        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            node.get_logger().error("goal rejected")
            return 1
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
        path = result_future.result().result.path
    finally:
        node.destroy_node()
        rclpy.shutdown()

    render(
        grid,
        resolution,
        origin,
        path,
        args.start,
        args.goal,
        args.planner_label,
        args.scenario,
        args.case,
        args.output,
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
