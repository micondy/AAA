#!/usr/bin/env python3
"""Drive the Isaac base through the safety gate and inject obstacles/dropout."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool


OBSTACLE_DISTANCE_M = 1.00
ROBOT_FRONT_M = 0.42
OBSTACLE_HALF_LENGTH_M = {"low": 0.225, "hanging": 0.20}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path, default=Path("results/navigation/intrusion")
    )
    parser.add_argument("--stop-latency-limit", type=float, default=1.0)
    parser.add_argument("--dropout-latency-limit", type=float, default=1.2)
    parser.add_argument("--minimum-clearance", type=float, default=0.05)
    return parser.parse_known_args()


class DynamicIntrusionBenchmark(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_intrusion_benchmark")
        self.declare_parameter("allow_motion", False)
        self.relay_enabled = True
        self.latest_odom: Odometry | None = None
        self.outputs: list[tuple[float, float]] = []
        self.states: list[tuple[float, int, str]] = []
        self.command_pub = self.create_publisher(Twist, "/safety_test/cmd_vel_in", 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/safety_test/points", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/sensors/nav_camera/depth/color/points",
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_output, 10)
        self.create_subscription(
            CollisionMonitorState, "/safety_test/state", self._on_state, 10
        )
        self.create_subscription(
            Odometry, "/swerve_controller/odom", self._on_odom, 20
        )
        self.obstacle_clients = {
            kind: self.create_client(
                SetBool, f"/sim/nav_obstacle/{kind}/set_enabled"
            )
            for kind in OBSTACLE_HALF_LENGTH_M
        }

    def _on_cloud(self, message: PointCloud2) -> None:
        if self.relay_enabled:
            self.cloud_pub.publish(message)

    def _on_output(self, message: Twist) -> None:
        self.outputs.append((time.monotonic(), float(message.linear.x)))

    def _on_state(self, message: CollisionMonitorState) -> None:
        self.states.append(
            (time.monotonic(), int(message.action_type), str(message.polygon_name))
        )

    def _on_odom(self, message: Odometry) -> None:
        self.latest_odom = message

    def set_obstacle(self, kind: str, enabled: bool) -> bool:
        future = self.obstacle_clients[kind].call_async(SetBool.Request(data=enabled))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.done() and future.result() and future.result().success)

    def command_for(self, speed: float, duration: float) -> None:
        command = Twist()
        command.linear.x = speed
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.command_pub.publish(command)
                next_publish = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop_command(self, duration: float = 0.6) -> None:
        self.command_for(0.0, duration)

    def pose_xy_yaw(self) -> tuple[float, float, float]:
        if self.latest_odom is None:
            raise RuntimeError("odometry unavailable")
        pose = self.latest_odom.pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        return float(pose.position.x), float(pose.position.y), yaw

    def run_intrusion(self, kind: str, speed: float, args) -> dict:
        self.relay_enabled = True
        if not self.set_obstacle(kind, False):
            raise RuntimeError(f"failed to clear {kind} obstacle")
        self.command_for(speed, 2.0)
        moving_before = bool(self.outputs and self.outputs[-1][1] >= 0.8 * speed)
        start_x, start_y, start_yaw = self.pose_xy_yaw()
        output_start = len(self.outputs)
        state_start = len(self.states)
        event_time = time.monotonic()
        if not self.set_obstacle(kind, True):
            raise RuntimeError(f"failed to place {kind} obstacle")
        self.command_for(speed, 2.0)
        self.stop_command()
        time.sleep(0.3)
        rclpy.spin_once(self, timeout_sec=0.05)
        end_x, end_y, _ = self.pose_xy_yaw()
        forward_travel = (
            math.cos(start_yaw) * (end_x - start_x)
            + math.sin(start_yaw) * (end_y - start_y)
        )
        outputs = self.outputs[output_start:]
        stop_times = [stamp for stamp, value in outputs if stamp >= event_time and abs(value) < 1e-3]
        stop_latency = stop_times[0] - event_time if stop_times else math.inf
        states = self.states[state_start:]
        front_stop = any(
            action == CollisionMonitorState.STOP and polygon == "FrontStop"
            for _, action, polygon in states
        )
        initial_clearance = (
            OBSTACLE_DISTANCE_M - OBSTACLE_HALF_LENGTH_M[kind] - ROBOT_FRONT_M
        )
        final_clearance = initial_clearance - max(0.0, forward_travel)
        self.set_obstacle(kind, False)
        record = {
            "name": f"{kind}_{speed:.2f}_mps",
            "obstacle_type": kind,
            "command_speed_mps": speed,
            "output_stop_latency_s": stop_latency,
            "forward_travel_after_intrusion_m": forward_travel,
            "estimated_final_clearance_m": final_clearance,
            "moving_before_intrusion": moving_before,
            "front_stop_seen": front_stop,
        }
        record["passed"] = (
            moving_before
            and front_stop
            and stop_latency <= args.stop_latency_limit
            and final_clearance >= args.minimum_clearance
        )
        return record

    def run_dropout(self, speed: float, args) -> dict:
        for kind in OBSTACLE_HALF_LENGTH_M:
            self.set_obstacle(kind, False)
        self.relay_enabled = True
        self.command_for(speed, 2.0)
        moving_before = bool(self.outputs and self.outputs[-1][1] >= 0.8 * speed)
        output_start = len(self.outputs)
        event_time = time.monotonic()
        self.relay_enabled = False
        self.command_for(speed, 2.0)
        self.stop_command()
        outputs = self.outputs[output_start:]
        stop_times = [stamp for stamp, value in outputs if stamp >= event_time and abs(value) < 1e-3]
        stop_latency = stop_times[0] - event_time if stop_times else math.inf
        self.relay_enabled = True
        record = {
            "name": "camera_dropout",
            "command_speed_mps": speed,
            "output_stop_latency_s": stop_latency,
            "moving_before_dropout": moving_before,
        }
        record["passed"] = moving_before and stop_latency <= args.dropout_latency_limit
        return record


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = DynamicIntrusionBenchmark()
    try:
        if not node.get_parameter("allow_motion").value:
            node.get_logger().error(
                "motion disabled: run the isolated Isaac simulation and pass "
                "allow_motion:=true explicitly"
            )
            return 2
        for kind, client in node.obstacle_clients.items():
            if not client.wait_for_service(timeout_sec=15.0):
                node.get_logger().error(f"missing {kind} obstacle service")
                return 2
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and (
            node.latest_odom is None
            or node.cloud_pub.get_subscription_count() == 0
            or node.command_pub.get_subscription_count() == 0
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest_odom is None or node.command_pub.get_subscription_count() == 0:
            node.get_logger().error("simulation or collision monitor is unavailable")
            return 2

        records = [
            node.run_intrusion("low", 0.10, args),
            node.run_intrusion("low", 0.18, args),
            node.run_intrusion("hanging", 0.18, args),
            node.run_dropout(0.10, args),
        ]
        passed = all(record["passed"] for record in records)
        result = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "passed": passed,
            "source_timeout_s": 0.75,
            "records": records,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        output = args.output_dir / (
            "dynamic_intrusion_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".json"
        )
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        for record in records:
            print(f"{record['name']}: passed={record['passed']} latency={record['output_stop_latency_s']:.3f}s")
        print(f"RESULT: {'PASS' if passed else 'FAIL'}")
        print(f"JSON: {output}")
        return 0 if passed else 1
    finally:
        node.stop_command(0.3)
        for kind in OBSTACLE_HALF_LENGTH_M:
            if node.obstacle_clients[kind].service_is_ready():
                node.set_obstacle(kind, False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
