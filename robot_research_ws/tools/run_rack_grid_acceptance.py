#!/usr/bin/env python3
"""Drive the 4x4 rack aisles and verify a complete persistent rack map."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from scipy.optimize import linear_sum_assignment


AISLE_X = (0.9, 3.5, 6.1, 8.7)
RACK_Y = (-5.1, -1.7, 1.7, 5.1)
END_Y = 6.5
TRUTH_RACKS = tuple((x, y) for x in (2.0, 4.6, 7.2, 9.8) for y in RACK_Y)


def route() -> tuple[tuple[float, float, bool], ...]:
    """Pass every rack front, turning the camera toward it at each stop."""
    points: list[tuple[float, float, bool]] = [(0.0, -END_Y, False)]
    for row, x in enumerate(AISLE_X):
        northbound = row % 2 == 0
        entry_y = -END_Y if northbound else END_Y
        points.append((x, entry_y, False))
        for y in (RACK_Y if northbound else reversed(RACK_Y)):
            points.append((x, y, True))
        points.append((x, END_Y if northbound else -END_Y, False))
    return tuple(points)


WAYPOINTS = route()


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class GridAcceptance(Node):
    def __init__(self, anonymous: bool = False) -> None:
        super().__init__("rack_grid_acceptance")
        self.anonymous = anonymous
        self.position: tuple[float, float] | None = None
        self.yaw = 0.0
        self.model: dict | None = None
        self.maximum_racks = 0
        self.maximum_layers = 0
        self.complete_id_sets: set[tuple[str, ...]] = set()
        self.truth_to_rack_ids: dict[int, set[str]] = defaultdict(set)
        self.rack_to_truth_ids: dict[str, set[int]] = defaultdict(set)
        self.truth_to_layer_ids: dict[tuple[int, int], set[str]] = defaultdict(set)
        self.model_frames = 0
        self.minimum_clearance_m = math.inf
        self.overlap_risk_samples = 0
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/swerve_controller/odom", self._odom, 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/rack_map/model", self._map, latched)

    def _odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.position = (pose.position.x, pose.position.y)
        q = pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        half_x = abs(math.cos(self.yaw)) * 0.327 + abs(math.sin(self.yaw)) * 0.300
        half_y = abs(math.sin(self.yaw)) * 0.327 + abs(math.cos(self.yaw)) * 0.300
        for rack_x in (2.0, 4.6, 7.2, 9.8):
            for rack_y in RACK_Y:
                gap_x = abs(pose.position.x - rack_x) - (0.40 + half_x)
                gap_y = abs(pose.position.y - rack_y) - (1.00 + half_y)
                clearance = (
                    math.hypot(max(gap_x, 0.0), max(gap_y, 0.0))
                    if gap_x >= 0.0 or gap_y >= 0.0
                    else max(gap_x, gap_y)
                )
                self.minimum_clearance_m = min(self.minimum_clearance_m, clearance)
                if clearance < 0.0:
                    self.overlap_risk_samples += 1

    def _map(self, message: String) -> None:
        try:
            model = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if model.get("schema") != "rack_map/v1":
            return
        self.model = model
        self.model_frames += 1
        racks = model.get("racks", [])
        if self.anonymous:
            for rack in racks:
                closest = min(
                    range(len(TRUTH_RACKS)),
                    key=lambda index: math.dist(rack["position"][:2], TRUTH_RACKS[index]),
                )
                if math.dist(rack["position"][:2], TRUTH_RACKS[closest]) > 0.75:
                    continue
                stable_id = rack["stable_id"]
                self.truth_to_rack_ids[closest].add(stable_id)
                self.rack_to_truth_ids[stable_id].add(closest)
                for layer in rack.get("layers", []):
                    self.truth_to_layer_ids[(closest, layer["level_number"])].add(layer["stable_id"])
        self.maximum_racks = max(self.maximum_racks, len(racks))
        self.maximum_layers = max(
            self.maximum_layers,
            sum(len(rack.get("layers", [])) for rack in racks),
        )
        if len(racks) == 16 and all(len(rack.get("layers", [])) == 3 for rack in racks):
            ids = tuple(
                sorted(
                    [rack["stable_id"] for rack in racks]
                    + [
                        layer["stable_id"]
                        for rack in racks
                        for layer in rack.get("layers", [])
                    ]
                )
            )
            self.complete_id_sets.add(ids)

    def stop(self) -> None:
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

    def face(self, target_yaw: float, deadline: float) -> bool:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.position is None:
                continue
            error = math.remainder(target_yaw - self.yaw, 2.0 * math.pi)
            if abs(error) <= 0.07:
                self.stop()
                return True
            command = Twist()
            command.angular.z = clamp(1.2 * error, 0.45)
            self.cmd_pub.publish(command)
        self.stop()
        return False

    def drive_to(self, target: tuple[float, float], deadline: float) -> bool:
        settled_since = None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.position is None:
                continue
            dx = target[0] - self.position[0]
            dy = target[1] - self.position[1]
            distance = math.hypot(dx, dy)
            if distance < 0.10:
                self.stop()
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= 0.5:
                    return True
                continue
            settled_since = None
            target_yaw = math.atan2(dy, dx)
            heading_error = math.remainder(target_yaw - self.yaw, 2.0 * math.pi)
            command = Twist()
            command.angular.z = clamp(1.2 * heading_error, 0.45)
            if abs(heading_error) < 0.25:
                command.linear.x = min(0.30, max(0.08, 0.8 * distance))
            self.cmd_pub.publish(command)
        self.stop()
        return False


def height_errors(model: dict | None) -> list[float]:
    if not model:
        return []
    expected = (0.55, 1.05, 1.55)
    errors = []
    for rack in model.get("racks", []):
        layers = sorted(rack.get("layers", []), key=lambda item: item["level_number"])
        if len(layers) == 3:
            errors.extend(abs(float(layer["height_m"]) - truth) for layer, truth in zip(layers, expected))
    return errors


def position_errors(model: dict | None, anonymous: bool = False) -> list[float]:
    if not model:
        return []
    errors = []
    if anonymous:
        estimates = model.get("racks", [])
        if not estimates:
            return []
        costs = [
            [math.dist(rack["position"][:2], truth) for truth in TRUTH_RACKS]
            for rack in estimates
        ]
        rows, cols = linear_sum_assignment(costs)
        return [costs[row][col] for row, col in zip(rows, cols)]
    expected_x = (2.0, 4.6, 7.2, 9.8)
    for rack in model.get("racks", []):
        source = rack.get("source_instance_id")
        if not isinstance(source, int) or not 1 <= source <= 16:
            continue
        row, column = divmod(source - 1, 4)
        errors.append(math.dist(rack["position"][:2], (expected_x[row], RACK_Y[column])))
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=660.0)
    parser.add_argument("--start-waypoint", type=int, default=0)
    parser.add_argument("--anonymous", action="store_true", help="score without Isaac source rack IDs")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/rack/grid_acceptance.json"),
    )
    args = parser.parse_args()
    if not 0 <= args.start_waypoint < len(WAYPOINTS):
        parser.error("--start-waypoint must be a zero-based index within the route")
    rclpy.init()
    node = GridAcceptance(anonymous=args.anonymous)
    start = time.monotonic()
    deadline = start + args.duration
    reached = args.start_waypoint
    interrupted = False
    try:
        for x, y, inspect in WAYPOINTS[args.start_waypoint:]:
            if not node.drive_to((x, y), deadline):
                break
            reached += 1
            if inspect:
                if not node.face(0.0, deadline):
                    break
                end_scan = time.monotonic() + 1.0
                while time.monotonic() < end_scan and rclpy.ok():
                    node.stop()
                    rclpy.spin_once(node, timeout_sec=0.05)
            print(
                f"[rack_grid_acceptance] waypoint={reached}/{len(WAYPOINTS)} "
                f"position=({x:.1f},{y:.1f}) "
                f"max_racks={node.maximum_racks} max_layers={node.maximum_layers}",
                flush=True,
            )
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.stop()
        if rclpy.ok():
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0.05)

    racks = node.model.get("racks", []) if node.model else []
    final_layers = sum(len(rack.get("layers", [])) for rack in racks)
    errors = height_errors(node.model)
    xy_errors = position_errors(node.model, anonymous=args.anonymous)
    rack_fragments = {index: sorted(ids) for index, ids in node.truth_to_rack_ids.items() if len(ids) > 1}
    rack_switches = {stable_id: sorted(ids) for stable_id, ids in node.rack_to_truth_ids.items() if len(ids) > 1}
    layer_fragments = {
        f"{index + 1}:{level}": sorted(ids)
        for (index, level), ids in node.truth_to_layer_ids.items() if len(ids) > 1
    }
    passed = (
        reached == len(WAYPOINTS)
        and len(racks) == 16
        and final_layers == 48
        and len(node.complete_id_sets) == 1
        and bool(errors)
        and max(errors) <= 0.05
        and len(xy_errors) == 16
        and max(xy_errors) <= 0.35
        and node.overlap_risk_samples == 0
        and (not args.anonymous or (
            len(node.truth_to_rack_ids) == 16
            and len(node.truth_to_layer_ids) == 48
            and not rack_fragments and not rack_switches and not layer_fragments
        ))
    )
    report = {
        "passed": passed,
        "anonymous_instances": args.anonymous,
        "interrupted": interrupted,
        "elapsed_wall_s": time.monotonic() - start,
        "waypoints_reached": reached,
        "waypoints_total": len(WAYPOINTS),
        "resumed_from_waypoint": args.start_waypoint,
        "model_frames": node.model_frames,
        "final_racks": len(racks),
        "final_layers": final_layers,
        "maximum_racks": node.maximum_racks,
        "maximum_layers": node.maximum_layers,
        "complete_stable_id_sets": len(node.complete_id_sets),
        "maximum_height_error_m": max(errors) if errors else None,
        "maximum_position_error_m": max(xy_errors) if xy_errors else None,
        "truth_racks_ever_associated": len(node.truth_to_rack_ids),
        "truth_layers_ever_associated": len(node.truth_to_layer_ids),
        "rack_id_fragments": rack_fragments,
        "rack_id_switches": rack_switches,
        "layer_id_fragments": layer_fragments,
        "minimum_clearance_m": node.minimum_clearance_m if math.isfinite(node.minimum_clearance_m) else None,
        "overlap_risk_samples": node.overlap_risk_samples,
        "final_position": list(node.position) if node.position else None,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
