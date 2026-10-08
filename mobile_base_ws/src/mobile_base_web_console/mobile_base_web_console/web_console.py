from __future__ import annotations

import hmac
import json
import math
import mimetypes
import os
import select
import socket
import struct
import subprocess
import threading
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from nvblox_msgs.msg import Mesh
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .chassis_config import ChassisConfigManager, WHEEL_NAMES
from .config import ConfigManager


def _json_safe(value: Any) -> Any:
    """把 ROS 状态中的 NaN/Inf 转为标准 JSON 的 null。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _finite_number(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数字")
    return number


def _run_length_encode(values: list[int]) -> list[int]:
    """把 OccupancyGrid 压缩为 [值, 数量, ...]，避免反复传输大 JSON 数组。"""
    if not values:
        return []
    encoded: list[int] = []
    current = int(values[0])
    count = 1
    for raw in values[1:]:
        value = int(raw)
        if value == current:
            count += 1
        else:
            encoded.extend((current, count))
            current, count = value, 1
    encoded.extend((current, count))
    return encoded


class WebConsoleNode(Node):
    def __init__(self, share_dir: Path):
        super().__init__("mobile_base_web_console")
        self.config_manager = ConfigManager(share_dir / "config" / "web_console.yaml")
        self.chassis_config_manager = ChassisConfigManager(share_dir / "config" / "chassis.yaml")
        config = self.config_manager.get()
        self.static_dir = share_dir / "web"
        self.auth_token = os.environ.get("MOBILE_BASE_WEB_TOKEN", "")
        self.started_at = time.monotonic()
        self.lock = threading.RLock()
        self.joints: dict[str, dict[str, float | None]] = {}
        self.odom: dict[str, float] = {}
        self.last_joint_time = 0.0
        self.last_odom_time = 0.0
        self.map_data: dict[str, Any] = {}
        self.last_map_time = 0.0
        self.map_revision = 0
        self.mesh_blocks: dict[tuple[int, int, int], Any] = {}
        self.mesh_block_size = 0.0
        self.mesh_frame = ""
        self.mesh_revision = 0
        self.last_mesh_time = 0.0
        self.map_pose: dict[str, float] = {}
        self.last_map_pose_time = 0.0
        self.navigation_goal_handle = None
        self.navigation_sequence = 0
        self.navigation = {"state": "IDLE", "message": "尚未发送导航目标"}
        self.last_command = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.maintenance_deadline = 0.0
        self.mode = "DISABLED"
        self.maintenance_command = {"wheel": None, "rpm": 0.0, "angle_deg": 0.0}
        self.calibration_previews: dict[str, dict[str, Any]] = {}
        self.controller_states: dict[str, str] = {}
        self.controller_query_future = None
        self.steering_jump_faults: dict[str, float] = {}
        self.can_status_cache: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self.simulation_direct_control = os.environ.get(
            "MOBILE_BASE_SIMULATION_DIRECT_CONTROL", ""
        ).strip().lower() in ("1", "true", "yes", "on")

        topics = config["topics"]
        self.command_publisher = self.create_publisher(Twist, topics["command"], 10)
        self.steering_maintenance_publisher = self.create_publisher(
            Float64MultiArray, "/steering_maintenance_controller/commands", 10
        )
        self.drive_maintenance_publisher = self.create_publisher(
            Float64MultiArray, "/drive_maintenance_controller/commands", 10
        )
        self.create_subscription(JointState, topics["joint_states"], self._on_joints, 10)
        self.create_subscription(Odometry, topics["odometry"], self._on_odom, 10)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, topics["map"], self._on_map, map_qos)
        self.create_subscription(Mesh, topics["nvblox_mesh"], self._on_mesh, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, topics["localization_pose"],
            self._on_localization_pose, 10,
        )
        self.navigation_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.list_controllers_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(1.0, self._poll_controllers)
        self.create_timer(1.0, self._publish_diagnostics)
        self._event("info", "网页控制台已启动")
        if self.simulation_direct_control:
            self._event("warn", "Isaac Sim 直控已启用；维护与硬件标定功能已禁用")

    def _on_joints(self, message: JointState) -> None:
        now = time.monotonic()
        result: dict[str, dict[str, float | None]] = {}
        for index, name in enumerate(message.name):
            result[name] = {
                "position": message.position[index] if index < len(message.position) else None,
                "velocity": message.velocity[index] if index < len(message.velocity) else None,
                "effort": message.effort[index] if index < len(message.effort) else None,
            }
        jump_limit = self.chassis_config_manager.get()["safety"]["steering_max_feedback_jump_rad"]
        with self.lock:
            for wheel_name in WHEEL_NAMES:
                joint_name = f"{wheel_name}_steer_joint"
                old = self.joints.get(joint_name, {}).get("position")
                new = result.get(joint_name, {}).get("position")
                if old is not None and new is not None and math.isfinite(old) and math.isfinite(new):
                    if abs(math.remainder(new - old, 2.0 * math.pi)) > jump_limit:
                        self.steering_jump_faults[joint_name] = now
            self.joints = result
            self.last_joint_time = now

    def _on_odom(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        with self.lock:
            self.odom = {
                "x": message.pose.pose.position.x,
                "y": message.pose.pose.position.y,
                "yaw": yaw,
                "vx": message.twist.twist.linear.x,
                "vy": message.twist.twist.linear.y,
                "wz": message.twist.twist.angular.z,
            }
            self.last_odom_time = time.monotonic()

    def _on_map(self, message: OccupancyGrid) -> None:
        orientation = message.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        width, height = int(message.info.width), int(message.info.height)
        expected = width * height
        if width <= 0 or height <= 0 or len(message.data) != expected:
            self.get_logger().warning("忽略尺寸不合法的 OccupancyGrid")
            return
        snapshot = {
            "width": width,
            "height": height,
            "resolution": float(message.info.resolution),
            "origin": {
                "x": float(message.info.origin.position.x),
                "y": float(message.info.origin.position.y),
                "yaw": origin_yaw,
            },
            "data_rle": _run_length_encode(message.data),
        }
        with self.lock:
            self.map_revision += 1
            snapshot["revision"] = self.map_revision
            self.map_data = snapshot
            self.last_map_time = time.monotonic()

    def map_snapshot(self, client_revision: int = -1) -> dict[str, Any]:
        with self.lock:
            if not self.map_data:
                return {"available": False, "revision": 0}
            age = max(0.0, time.monotonic() - self.last_map_time)
            revision = self.map_revision
            if client_revision == revision:
                return {"available": True, "revision": revision, "unchanged": True, "age": age}
            result = dict(self.map_data)
            result.update({"available": True, "unchanged": False, "age": age})
            return result

    def _on_mesh(self, message: Mesh) -> None:
        """Cache incremental nvblox mesh blocks for the lightweight web viewer."""
        with self.lock:
            if message.clear:
                self.mesh_blocks.clear()
            self.mesh_block_size = float(message.block_size_m)
            self.mesh_frame = message.header.frame_id
            for index, block in zip(message.block_indices, message.blocks):
                key = (int(index.x), int(index.y), int(index.z))
                if block.vertices:
                    self.mesh_blocks[key] = block
                else:
                    self.mesh_blocks.pop(key, None)
            self.mesh_revision += 1
            self.last_mesh_time = time.monotonic()

    def mesh_snapshot(self, client_revision: int = -1, max_points: int = 8000) -> dict[str, Any]:
        """Return a bounded colored vertex cloud derived from the nvblox mesh."""
        with self.lock:
            if not self.mesh_blocks:
                return {"available": False, "revision": self.mesh_revision}
            age = max(0.0, time.monotonic() - self.last_mesh_time)
            revision = self.mesh_revision
            if client_revision == revision:
                return {"available": True, "revision": revision, "unchanged": True, "age": age}
            blocks = list(self.mesh_blocks.items())
            frame = self.mesh_frame
        total = sum(len(block.vertices) for _, block in blocks)
        stride = max(1, math.ceil(total / max(1, max_points)))
        points: list[float | int] = []
        cursor = 0
        for _block_index, block in blocks:
            colors = block.colors
            for vertex_index, vertex in enumerate(block.vertices):
                if cursor % stride == 0:
                    color = colors[vertex_index] if vertex_index < len(colors) else None
                    points.extend((
                        # nvblox_msgs/Mesh vertices are already expressed in
                        # header.frame_id. block_indices identify incremental
                        # replacement blocks; they are not an extra transform.
                        round(float(vertex.x), 3),
                        round(float(vertex.y), 3),
                        round(float(vertex.z), 3),
                        int(max(0, min(255, round((color.r if color else 0.45) * 255)))),
                        int(max(0, min(255, round((color.g if color else 0.85) * 255)))),
                        int(max(0, min(255, round((color.b if color else 0.65) * 255)))),
                    ))
                cursor += 1
        return {
            "available": True, "unchanged": False, "revision": revision, "age": age,
            "frame_id": frame, "block_count": len(blocks), "vertex_count": total,
            "sample_count": len(points) // 6, "points": points,
        }

    def _on_localization_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        with self.lock:
            self.map_pose = {"x": float(pose.position.x), "y": float(pose.position.y), "yaw": yaw}
            self.last_map_pose_time = time.monotonic()

    def navigate_to(self, body: dict[str, Any]) -> dict[str, Any]:
        x = _finite_number(body.get("x"), "x")
        y = _finite_number(body.get("y"), "y")
        yaw = _finite_number(body.get("yaw", 0.0), "yaw")
        if not self.navigation_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError("Nav2 /navigate_to_pose 当前不可用")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        with self.lock:
            self.navigation_sequence += 1
            sequence = self.navigation_sequence
            self.navigation = {
                "state": "SENDING", "message": "正在提交导航目标",
                "goal": {"x": x, "y": y, "yaw": yaw},
            }
        future = self.navigation_client.send_goal_async(
            goal, feedback_callback=lambda feedback: self._navigation_feedback(sequence, feedback)
        )
        future.add_done_callback(lambda result: self._navigation_goal_response(sequence, result))
        return {"accepted": True, "state": "SENDING", "goal": {"x": x, "y": y, "yaw": yaw}}

    def _navigation_feedback(self, sequence: int, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        with self.lock:
            if sequence != self.navigation_sequence:
                return
            self.navigation.update({
                "state": "ACTIVE", "message": "正在导航",
                "distance_remaining": float(feedback.distance_remaining),
                "navigation_time": float(feedback.navigation_time.sec) + float(feedback.navigation_time.nanosec) / 1e9,
            })

    def _navigation_goal_response(self, sequence: int, future: Any) -> None:
        try:
            handle = future.result()
            with self.lock:
                if sequence != self.navigation_sequence:
                    return
                if not handle.accepted:
                    self.navigation.update({"state": "REJECTED", "message": "Nav2 拒绝了目标"})
                    return
                self.navigation_goal_handle = handle
                self.navigation.update({"state": "ACTIVE", "message": "Nav2 已接受目标"})
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda result: self._navigation_result(sequence, result))
        except Exception as error:
            with self.lock:
                if sequence == self.navigation_sequence:
                    self.navigation.update({"state": "ERROR", "message": str(error)})

    def _navigation_result(self, sequence: int, future: Any) -> None:
        labels = {4: ("SUCCEEDED", "已到达目标"), 5: ("CANCELED", "导航已取消"), 6: ("ABORTED", "导航失败")}
        try:
            status = int(future.result().status)
            state, message = labels.get(status, ("FINISHED", f"导航结束（状态 {status}）"))
        except Exception as error:
            state, message = "ERROR", str(error)
        with self.lock:
            if sequence == self.navigation_sequence:
                self.navigation.update({"state": state, "message": message})
                self.navigation_goal_handle = None

    def cancel_navigation(self) -> dict[str, Any]:
        with self.lock:
            handle = self.navigation_goal_handle
            self.navigation_sequence += 1
            self.navigation = {"state": "CANCELED", "message": "已请求取消导航"}
            self.navigation_goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()
        self._publish(0.0, 0.0, 0.0)
        return {"canceled": True}

    def _event(self, level: str, message: str) -> None:
        with self.lock:
            self.events.append({"time": time.strftime("%H:%M:%S"), "level": level, "message": message})
            self.events = self.events[-30:]

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = vx, vy, wz
        # Ctrl+C 可能先使 ROS context 失效，此时只更新本地状态，避免退出时报二次异常。
        if rclpy.ok():
            self.command_publisher.publish(message)
        with self.lock:
            self.last_command = {"vx": vx, "vy": vy, "wz": wz}

    def stop(self, reason: str) -> None:
        self._publish(0.0, 0.0, 0.0)
        self._publish_maintenance_stop()
        with self.lock:
            self.maintenance_deadline = 0.0
        self._event("warn", reason)

    def _watchdog(self) -> None:
        with self.lock:
            expired = bool(
                self.maintenance_deadline and
                time.monotonic() > self.maintenance_deadline
            )
        if expired:
            self.stop("维护命令超时，已发送零速度")

    def _publish_maintenance_stop(self) -> None:
        drive_message = Float64MultiArray()
        drive_message.data = [0.0] * len(WHEEL_NAMES)
        with self.lock:
            joints = dict(self.joints)
        steering_message = Float64MultiArray()
        steering_message.data = [
            float(joints.get(f"{name}_steer_joint", {}).get("position") or 0.0)
            for name in WHEEL_NAMES
        ]
        if rclpy.ok():
            self.steering_maintenance_publisher.publish(steering_message)
            self.drive_maintenance_publisher.publish(drive_message)
        with self.lock:
            self.maintenance_command["rpm"] = 0.0
            self.maintenance_deadline = 0.0

    @staticmethod
    def _await_future(future, timeout: float):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError("controller_manager 服务响应超时")
        result = future.result()
        if result is None:
            raise RuntimeError("controller_manager 服务调用失败")
        return result

    def _list_controllers(self, timeout: float = 1.0) -> dict[str, str]:
        if not self.list_controllers_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("controller_manager/list_controllers 不可用")
        response = self._await_future(
            self.list_controllers_client.call_async(ListControllers.Request()), timeout
        )
        states = {controller.name: controller.state for controller in response.controller}
        with self.lock:
            self.controller_states = states
        return states

    def _poll_controllers(self) -> None:
        if getattr(self, "simulation_direct_control", False):
            return
        if not self.list_controllers_client.service_is_ready():
            return
        if self.controller_query_future is not None and not self.controller_query_future.done():
            return
        future = self.list_controllers_client.call_async(ListControllers.Request())
        self.controller_query_future = future

        def complete(done):
            try:
                response = done.result()
                states = {controller.name: controller.state for controller in response.controller}
                with self.lock:
                    self.controller_states = states
                    if states.get("swerve_controller") == "active":
                        self.mode = "NORMAL"
                    elif (
                        states.get("steering_maintenance_controller") == "active" and
                        states.get("drive_maintenance_controller") == "active"
                    ):
                        self.mode = "MAINTENANCE"
                    elif not any(states.get(name) == "active" for name in (
                        "swerve_controller", "steering_maintenance_controller",
                        "drive_maintenance_controller",
                    )):
                        self.mode = "DISABLED"
                    else:
                        # 维护控制器只激活一半也属于不安全状态，不保留旧模式。
                        self.mode = "DISABLED"
            except Exception as error:
                self.get_logger().warn(f"读取控制器状态失败：{error}")

        future.add_done_callback(complete)

    def _switch_controllers(self, activate: list[str], deactivate: list[str]) -> None:
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("controller_manager/switch_controller 不可用")
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 2
        response = self._await_future(self.switch_controller_client.call_async(request), 3.0)
        if not response.ok:
            raise RuntimeError(response.message or "controller_manager 拒绝切换控制器")

    def _steering_feedback_health(self) -> dict[str, Any]:
        now = time.monotonic()
        timeout = self.chassis_config_manager.get()["safety"]["steering_feedback_timeout"]
        reasons: list[str] = []
        with self.lock:
            joints = dict(self.joints)
            age = now - self.last_joint_time if self.last_joint_time else None
            jumps = dict(self.steering_jump_faults)
        if age is None or age > timeout:
            reasons.append("/joint_states 转向反馈超时")
        for wheel_name in WHEEL_NAMES:
            joint_name = f"{wheel_name}_steer_joint"
            position = joints.get(joint_name, {}).get("position")
            if position is None:
                reasons.append(f"{joint_name} 无位置反馈")
            elif not math.isfinite(position):
                reasons.append(f"{joint_name} 位置非有限值")
            if now - jumps.get(joint_name, 0.0) < 2.0:
                reasons.append(f"{joint_name} 检测到角度跳变")
        return {"healthy": not reasons, "reasons": reasons, "age": age}

    def set_mode(self, body: dict[str, Any]) -> dict[str, Any]:
        requested = str(body.get("mode", "")).upper()
        if requested not in ("DISABLED", "MAINTENANCE", "NORMAL"):
            raise ValueError("模式只能是 DISABLED、MAINTENANCE 或 NORMAL")
        if getattr(self, "simulation_direct_control", False):
            if requested == "MAINTENANCE":
                raise RuntimeError("Isaac Sim 直控模式不允许进入硬件维护模式")
            self.stop(f"正在切换到 {requested} 模式")
            with self.lock:
                self.mode = requested
                self.controller_states = {
                    "isaac_sim_direct_control": (
                        "active" if requested == "NORMAL" else "inactive"
                    )
                }
            self._event("info", f"Isaac Sim 逻辑工作模式已切换为 {requested}")
            return {"mode": requested, "controllers": dict(self.controller_states)}
        if requested == "MAINTENANCE":
            if body.get("acknowledge_wheels_lifted") is not True:
                raise RuntimeError("进入维护模式前必须确认轮子已架空且现场有人监护")
            health = self._steering_feedback_health()
            if not health["healthy"]:
                raise RuntimeError("转向反馈联锁拒绝维护模式：" + "；".join(health["reasons"]))
        self.stop(f"正在切换到 {requested} 模式")
        try:
            current = self._list_controllers()
            if requested == "MAINTENANCE":
                activate = [name for name in ("steering_maintenance_controller", "drive_maintenance_controller")
                            if current.get(name) != "active"]
                deactivate = ["swerve_controller"] if current.get("swerve_controller") == "active" else []
                if activate or deactivate:
                    self._switch_controllers(activate, deactivate)
            elif requested == "NORMAL":
                activate = ["swerve_controller"] if current.get("swerve_controller") != "active" else []
                deactivate = [name for name in ("steering_maintenance_controller", "drive_maintenance_controller")
                              if current.get(name) == "active"]
                if activate or deactivate:
                    self._switch_controllers(activate, deactivate)
            else:
                active = [name for name in (
                    "swerve_controller", "steering_maintenance_controller",
                    "drive_maintenance_controller",
                ) if current.get(name) == "active"]
                if active:
                    self._switch_controllers([], active)
            states = self._list_controllers()
            expected = {
                "NORMAL": states.get("swerve_controller") == "active",
                "MAINTENANCE": (
                    states.get("steering_maintenance_controller") == "active" and
                    states.get("drive_maintenance_controller") == "active"
                ),
                "DISABLED": not any(states.get(name) == "active" for name in (
                    "swerve_controller", "steering_maintenance_controller",
                    "drive_maintenance_controller",
                )),
            }[requested]
            if not expected:
                raise RuntimeError("控制器最终状态与请求模式不一致")
        except Exception:
            with self.lock:
                self.mode = "DISABLED"
            self._publish_maintenance_stop()
            raise
        with self.lock:
            self.mode = requested
        self._event("info", f"逻辑工作模式已切换为 {requested}")
        return {"mode": requested, "controllers": states}

    def maintenance_command_request(self, body: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if self.mode != "MAINTENANCE":
                raise RuntimeError("只有 MAINTENANCE 模式允许单轮控制")
        health = self._steering_feedback_health()
        if not health["healthy"]:
            self._publish_maintenance_stop()
            raise RuntimeError("转向反馈联锁：" + "；".join(health["reasons"]))
        wheel_name = str(body.get("wheel", ""))
        if wheel_name not in WHEEL_NAMES:
            raise ValueError("轮组名称无效")
        if body.get("deadman") is not True:
            self._publish_maintenance_stop()
            return {"accepted": True, "stopped": True}

        config = self.chassis_config_manager.get()
        safety = config["safety"]
        rpm = max(-safety["maintenance_max_rpm"], min(
            safety["maintenance_max_rpm"], _finite_number(body.get("rpm", 0.0), "rpm")
        ))
        index = WHEEL_NAMES.index(wheel_name)
        steering_values: list[float] = []
        with self.lock:
            joints = dict(self.joints)
        for name in WHEEL_NAMES:
            joint = joints.get(f"{name}_steer_joint", {})
            steering_values.append(float(joint.get("position") or 0.0))
        angle_mode = str(body.get("angle_mode", "relative")).lower()
        if angle_mode not in ("relative", "absolute"):
            raise ValueError("转角模式只能是 relative 或 absolute")
        command_id = str(body.get("command_id", ""))[:80]
        if not command_id:
            raise ValueError("缺少维护命令编号")
        with self.lock:
            previous = dict(self.maintenance_command)
        if previous.get("command_id") == command_id and previous.get("wheel") == wheel_name:
            angle_deg = float(previous["angle_deg"])
            requested_angle_deg = float(previous.get("requested_angle_deg", angle_deg))
            input_angle_deg = float(previous.get("input_angle_deg", angle_deg))
            clamped = bool(previous.get("clamped", False))
        else:
            # 同一次按住只在首帧计算一次目标，后续100ms刷新绝不重复累加相对角。
            current_angle_deg = math.degrees(steering_values[index])
            if angle_mode == "absolute":
                input_angle_deg = _finite_number(
                    body.get("target_angle_deg", 0.0), "target_angle_deg")
                requested_angle_deg = input_angle_deg
            else:
                step_limit = safety["maintenance_max_steering_step_deg"]
                raw_relative = _finite_number(
                    body.get("relative_angle_deg", 0.0), "relative_angle_deg")
                input_angle_deg = max(-step_limit, min(step_limit, raw_relative))
                requested_angle_deg = current_angle_deg + input_angle_deg
            absolute_limit = min(
                safety["steering_limit_deg"], safety["maintenance_max_steering_angle_deg"])
            angle_deg = max(-absolute_limit, min(absolute_limit, requested_angle_deg))
            clamped = not math.isclose(angle_deg, requested_angle_deg, abs_tol=0.01)
        absolute_limit = min(
            safety["steering_limit_deg"], safety["maintenance_max_steering_angle_deg"])
        angle_deg = max(-absolute_limit, min(absolute_limit, angle_deg))
        steering_values[index] = math.radians(angle_deg)
        drive_values = [0.0] * len(WHEEL_NAMES)
        drive_values[index] = rpm * 2.0 * math.pi / 60.0
        steering_message, drive_message = Float64MultiArray(), Float64MultiArray()
        steering_message.data, drive_message.data = steering_values, drive_values
        if rclpy.ok():
            self.steering_maintenance_publisher.publish(steering_message)
            self.drive_maintenance_publisher.publish(drive_message)
        with self.lock:
            self.maintenance_deadline = time.monotonic() + safety["command_timeout"]
            self.maintenance_command = {
                "wheel": wheel_name, "rpm": rpm, "angle_deg": angle_deg,
                "angle_mode": angle_mode, "requested_angle_deg": requested_angle_deg,
                "input_angle_deg": input_angle_deg, "clamped": clamped,
                "command_id": command_id}
        return {"accepted": True, "command": dict(self.maintenance_command)}

    def command(self, body: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if self.mode != "NORMAL":
                raise RuntimeError("只有 NORMAL 模式允许整车 cmd_vel 控制")
        if body.get("deadman") is not True:
            self._publish(0.0, 0.0, 0.0)
            return {"accepted": True, "stopped": True}
        safety = self.config_manager.get()["safety"]
        vx = max(-safety["max_linear_x"], min(safety["max_linear_x"], _finite_number(body.get("vx", 0), "vx")))
        vy = max(-safety["max_linear_y"], min(safety["max_linear_y"], _finite_number(body.get("vy", 0), "vy")))
        wz = max(-safety["max_angular_z"], min(safety["max_angular_z"], _finite_number(body.get("wz", 0), "wz")))
        self._publish(vx, vy, wz)
        return {"accepted": True, "command": self.last_command}

    def update_config(self, body: dict[str, Any]) -> dict[str, Any]:
        config, restart_required = self.config_manager.update(body)
        self._event("info", "配置已保存" + ("，重启后生效" if restart_required else ""))
        return {"config": config, "restart_required": restart_required}

    def update_chassis_config(self, body: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if self.mode != "DISABLED":
                raise RuntimeError("只有 DISABLED 模式允许保存底盘硬件配置")
        result = self.chassis_config_manager.save(body)
        self._event("info", "底盘配置已校验并保存；重启底盘硬件后生效")
        result["restart_required"] = True
        return result

    def read_steering_absolute(self, body: dict[str, Any]) -> dict[str, Any]:
        """只读达妙 0x51；不使能、不发位置命令，也不修改配置。"""
        with self.lock:
            if self.mode != "DISABLED":
                raise RuntimeError("只有 DISABLED 模式允许读取和标定机械零位")
        wheel_name = str(body.get("wheel", ""))
        if wheel_name not in WHEEL_NAMES:
            raise ValueError("轮组名称无效")
        config = self.chassis_config_manager.get()
        steering = config["wheels"][wheel_name]["steering"]
        interface = config["can"]["steering"]["interface"]
        motor_id, master_id = int(steering["motor_id"]), int(steering["master_id"])
        can_frame = struct.Struct("=IB3x8s")
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            sock.bind((interface,))
            request = bytes((motor_id, 0, 0x33, 0x51, 0, 0, 0, 0))
            sock.send(can_frame.pack(0x7FF, 8, request))
            deadline = time.monotonic() + 0.5
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        f"等待 ESC=0x{motor_id:02X}、MST=0x{master_id:03X} 的 0x51 回复超时"
                    )
                readable, _, _ = select.select([sock], [], [], remaining)
                if not readable:
                    continue
                can_id, length, payload = can_frame.unpack(sock.recv(can_frame.size))
                reply_motor = payload[0] | (payload[1] << 8)
                if (
                    (can_id & 0x7FF) == master_id and length >= 8 and
                    reply_motor == motor_id and payload[2] == 0x33 and payload[3] == 0x51
                ):
                    raw_position = float(struct.unpack("<f", payload[4:8])[0])
                    break
        finally:
            sock.close()
        if not math.isfinite(raw_position):
            raise RuntimeError("电机返回了无效的绝对位置")
        direction = int(steering["direction"])
        proposed_offset = -raw_position * direction
        preview_token = uuid.uuid4().hex
        with self.lock:
            self.calibration_previews = {
                key: value for key, value in self.calibration_previews.items()
                if time.monotonic() - value["created_at"] < 60.0
            }
            self.calibration_previews[preview_token] = {
                "wheel": wheel_name,
                "raw_position": raw_position,
                "proposed_offset": proposed_offset,
                "created_at": time.monotonic(),
            }
        self._event("info", f"已只读获取 {wheel_name} 输出轴绝对位置")
        return {
            "wheel": wheel_name, "motor_id": motor_id, "master_id": master_id,
            "raw_position": raw_position, "old_offset": float(steering["offset_rad"]),
            "proposed_offset": proposed_offset, "preview_token": preview_token,
        }

    def save_steering_zero(self, body: dict[str, Any]) -> dict[str, Any]:
        if body.get("confirm_wheel_is_straight") is not True:
            raise RuntimeError("必须确认该轮已经机械对正")
        with self.lock:
            if self.mode != "DISABLED":
                raise RuntimeError("只有 DISABLED 模式允许保存机械零位")
            preview = self.calibration_previews.pop(str(body.get("preview_token", "")), None)
        if not preview or time.monotonic() - preview["created_at"] >= 60.0:
            raise RuntimeError("标定预览不存在或已超过 60 秒，请重新读取")
        config = self.chassis_config_manager.get()
        config["wheels"][preview["wheel"]]["steering"]["offset_rad"] = preview["proposed_offset"]
        result = self.chassis_config_manager.save(config)
        self._event("warn", f"已保存 {preview['wheel']} 机械零位，重启底盘后生效")
        return {
            "wheel": preview["wheel"], "offset_rad": preview["proposed_offset"],
            "backup": result["backup"], "restart_required": True,
        }

    @staticmethod
    def _can_status(interface: str) -> dict[str, Any]:
        try:
            process = subprocess.run(
                ["ip", "-json", "-details", "-statistics", "link", "show", interface],
                capture_output=True, text=True, timeout=0.5, check=False,
            )
            if process.returncode != 0:
                return {"interface": interface, "available": False, "error": process.stderr.strip()}
            data = json.loads(process.stdout)[0]
            info = data.get("linkinfo", {}).get("info_data", {})
            return {
                "interface": interface,
                "available": True,
                "up": "UP" in data.get("flags", []),
                "operstate": data.get("operstate", "UNKNOWN"),
                "can_state": info.get("state", "UNKNOWN"),
                "bitrate": info.get("bitrate"),
                "tx_errors": info.get("berr_counter", {}).get("tx"),
                "rx_errors": info.get("berr_counter", {}).get("rx"),
            }
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            return {"interface": interface, "available": False, "error": str(error)}

    @staticmethod
    def _diagnostic_status(level: int, name: str, message: str, values: dict[str, Any]):
        status = DiagnosticStatus()
        status.level, status.name, status.message = level, name, message
        status.hardware_id = "mobile_base"
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        return status

    def _publish_diagnostics(self) -> None:
        health = self._steering_feedback_health()
        with self.lock:
            mode, states, can_status = self.mode, dict(self.controller_states), list(self.can_status_cache)
        statuses = [self._diagnostic_status(
            DiagnosticStatus.OK if health["healthy"] else DiagnosticStatus.ERROR,
            "Mobile Base/SteeringFeedback",
            "四轮转向反馈有效" if health["healthy"] else "；".join(health["reasons"]),
            {"age_seconds": health["age"], "wheel_count": 4},
        )]
        expected_active = {
            "NORMAL": ["swerve_controller"],
            "MAINTENANCE": ["steering_maintenance_controller", "drive_maintenance_controller"],
            "DISABLED": [],
        }[mode]
        controller_ok = bool(states) and all(states.get(name) == "active" for name in expected_active)
        if mode == "DISABLED" and states:
            controller_ok = not any(states.get(name) == "active" for name in (
                "swerve_controller", "steering_maintenance_controller", "drive_maintenance_controller"
            ))
        statuses.append(self._diagnostic_status(
            DiagnosticStatus.OK if controller_ok else DiagnosticStatus.WARN,
            "Mobile Base/ControllerMode", "控制器状态匹配" if controller_ok else "控制器状态未知或不匹配",
            {"mode": mode, **states},
        ))
        if not can_status:
            statuses.append(self._diagnostic_status(
                DiagnosticStatus.STALE, "Mobile Base/CAN", "尚未采集 CAN 状态", {}
            ))
        for item in can_status:
            healthy = item.get("available") and item.get("up") and item.get("can_state") != "BUS-OFF"
            statuses.append(self._diagnostic_status(
                DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
                f"Mobile Base/CAN/{item['interface']}",
                item.get("can_state") or item.get("error") or "状态未知", item,
            ))
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = statuses
        self.diagnostics_publisher.publish(message)

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        config = self.config_manager.get()
        chassis_config = self.chassis_config_manager.get()
        interfaces = sorted({item["interface"] for item in chassis_config["can"].values()})
        with self.lock:
            status = {
                "uptime": now - self.started_at,
                "authenticated": bool(self.auth_token),
                "joint_states": {
                    "online": bool(self.last_joint_time and now - self.last_joint_time < 1.0),
                    "age": now - self.last_joint_time if self.last_joint_time else None,
                    "joints": self.joints,
                },
                "odometry": {
                    "online": bool(self.last_odom_time and now - self.last_odom_time < 1.0),
                    "age": now - self.last_odom_time if self.last_odom_time else None,
                    "value": self.odom,
                },
                "map_pose": {
                    "online": bool(self.last_map_pose_time and now - self.last_map_pose_time < 2.0),
                    "age": now - self.last_map_pose_time if self.last_map_pose_time else None,
                    "value": self.map_pose,
                },
                "navigation": dict(self.navigation),
                "last_command": self.last_command,
                "mode": self.mode,
                "maintenance_command": dict(self.maintenance_command),
                "events": list(self.events),
            }
        status["can"] = [self._can_status(name) for name in interfaces]
        with self.lock:
            self.can_status_cache = list(status["can"])
        status["steering_feedback"] = self._steering_feedback_health()
        status["controllers"] = dict(self.controller_states)
        return status


class ConsoleHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], node: WebConsoleNode):
        self.node = node
        super().__init__(address, ConsoleRequestHandler)


class ConsoleRequestHandler(BaseHTTPRequestHandler):
    server: ConsoleHTTPServer

    def log_message(self, fmt: str, *args: Any) -> None:
        self.server.node.get_logger().debug(fmt % args)

    def _authorized(self) -> bool:
        required = self.server.node.auth_token
        if not required:
            return True
        supplied = self.headers.get("Authorization", "")
        return supplied.startswith("Bearer ") and hmac.compare_digest(supplied[7:], required)

    def _json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length > 65536:
            raise ValueError("请求内容过大")
        value = json.loads(self.rfile.read(length) or b"{}")
        if not isinstance(value, dict):
            raise ValueError("请求内容必须是 JSON 对象")
        return value

    def _send_json(self, status: int, value: Any) -> None:
        payload = json.dumps(
            _json_safe(value), ensure_ascii=False, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'")
        self.end_headers()
        self.wfile.write(payload)

    def _error(self, status: int, message: str) -> None:
        self._send_json(status, {"error": message})

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/api/status":
            self._send_json(HTTPStatus.OK, self.server.node.status())
            return
        if path == "/api/map":
            try:
                revision = int(parse_qs(urlparse(self.path).query).get("revision", ["-1"])[0])
            except ValueError:
                revision = -1
            self._send_json(HTTPStatus.OK, self.server.node.map_snapshot(revision))
            return
        if path == "/api/scene3d":
            try:
                revision = int(parse_qs(urlparse(self.path).query).get("revision", ["-1"])[0])
            except ValueError:
                revision = -1
            self._send_json(HTTPStatus.OK, self.server.node.mesh_snapshot(revision))
            return
        if path == "/api/config":
            self._send_json(HTTPStatus.OK, self.server.node.config_manager.get())
            return
        if path == "/api/chassis":
            self._send_json(HTTPStatus.OK, self.server.node.chassis_config_manager.get())
            return
        static_files = {
            "/": "index.html", "/index.html": "index.html", "/app.css": "app.css",
            "/maintenance.css": "maintenance.css", "/visualization.css": "visualization.css", "/app.js": "app.js",
        }
        if path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        filename = static_files.get(path)
        if not filename:
            self._error(HTTPStatus.NOT_FOUND, "页面不存在")
            return
        payload = (self.server.node.static_dir / filename).read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mimetypes.guess_type(filename)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(payload)

    def _write_request(self, method: str) -> None:
        if not self._authorized():
            self._error(HTTPStatus.UNAUTHORIZED, "访问令牌错误")
            return
        path = urlparse(self.path).path
        try:
            body = self._json_body()
            node = self.server.node
            if method == "PUT" and path == "/api/config":
                result = node.update_config(body)
            elif method == "PUT" and path == "/api/chassis":
                result = node.update_chassis_config(body)
            elif method == "POST" and path == "/api/control/cmd_vel":
                result = node.command(body)
            elif method == "POST" and path == "/api/control/stop":
                node.cancel_navigation()
                node.stop("网页端执行全局停车")
                result = {"stopped": True}
            elif method == "POST" and path == "/api/navigation/goal":
                result = node.navigate_to(body)
            elif method == "POST" and path == "/api/navigation/cancel":
                result = node.cancel_navigation()
            elif method == "POST" and path == "/api/mode":
                result = node.set_mode(body)
            elif method == "POST" and path == "/api/maintenance/command":
                result = node.maintenance_command_request(body)
            elif method == "POST" and path == "/api/calibration/read":
                result = node.read_steering_absolute(body)
            elif method == "POST" and path == "/api/calibration/save":
                result = node.save_steering_zero(body)
            else:
                self._error(HTTPStatus.NOT_FOUND, "接口不存在")
                return
            self._send_json(HTTPStatus.OK, result)
        except RuntimeError as error:
            self._error(HTTPStatus.CONFLICT, str(error))
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
            self._error(HTTPStatus.BAD_REQUEST, str(error))
        except Exception as error:  # 保证控制台返回可读错误，同时把详情写入 ROS 日志。
            self.server.node.get_logger().error(f"网页请求失败：{error}")
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, str(error))

    def do_POST(self) -> None:  # noqa: N802
        self._write_request("POST")

    def do_PUT(self) -> None:  # noqa: N802
        self._write_request("PUT")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    share_dir = Path(get_package_share_directory("mobile_base_web_console"))
    node = WebConsoleNode(share_dir)
    config = node.config_manager.get()
    address = (config["web"]["host"], config["web"]["port"])
    server = ConsoleHTTPServer(address, node)
    thread = threading.Thread(target=server.serve_forever, name="mobile-base-http", daemon=True)
    thread.start()
    node.get_logger().info(f"网页控制台：http://{address[0]}:{address[1]}")
    if not node.auth_token:
        node.get_logger().warning("未设置 MOBILE_BASE_WEB_TOKEN：局域网内任何设备都可访问控制接口")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop("网页控制台关闭")
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
