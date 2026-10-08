import math
import threading
import time
from types import SimpleNamespace

import pytest

from mobile_base_web_console.web_console import WebConsoleNode, _json_safe, _run_length_encode



def test_json_safe_replaces_non_finite_ros_values():
    value = _json_safe({"effort": math.nan, "values": [math.inf, -math.inf, 1.0]})
    assert value == {"effort": None, "values": [None, None, 1.0]}


def test_occupancy_grid_run_length_encoding():
    assert _run_length_encode([]) == []
    assert _run_length_encode([-1, -1, 0, 0, 0, 100]) == [-1, 2, 0, 3, 100, 1]


def test_map_snapshot_skips_unchanged_payload():
    node = WebConsoleNode.__new__(WebConsoleNode)
    node.lock = threading.RLock()
    node.map_revision = 4
    node.last_map_time = time.monotonic()
    node.map_data = {"revision": 4, "width": 2, "height": 1, "data_rle": [0, 2]}
    full = node.map_snapshot()
    assert full["available"] and full["data_rle"] == [0, 2]
    unchanged = node.map_snapshot(4)
    assert unchanged["unchanged"] and "data_rle" not in unchanged


def test_nvblox_mesh_snapshot_keeps_global_vertices_and_limits_payload():
    node = WebConsoleNode.__new__(WebConsoleNode)
    node.lock = threading.RLock()
    node.mesh_blocks = {
        (2, -1, 1): SimpleNamespace(
            vertices=[SimpleNamespace(x=0.1, y=0.2, z=0.3)],
            colors=[SimpleNamespace(r=1.0, g=0.5, b=0.0)],
        )
    }
    node.mesh_block_size = 0.4
    node.mesh_frame = "odom"
    node.mesh_revision = 3
    node.last_mesh_time = time.monotonic()
    result = node.mesh_snapshot()
    assert result["available"] and result["frame_id"] == "odom"
    assert result["points"] == pytest.approx([0.1, 0.2, 0.3, 255, 128, 0])
    assert node.mesh_snapshot(3)["unchanged"]

class Config:
    def get(self):
        return {"safety": {"steering_feedback_timeout": 0.5, "command_timeout": 0.5,
                            "maintenance_max_rpm": 20.0,
                            "steering_limit_deg": 90.0,
                            "maintenance_max_steering_angle_deg": 90.0,
                            "maintenance_max_steering_step_deg": 180.0}}


def bare_node():
    node = WebConsoleNode.__new__(WebConsoleNode)
    node.lock = threading.RLock()
    node.chassis_config_manager = Config()
    node.last_joint_time = time.monotonic()
    node.steering_jump_faults = {}
    node.joints = {}
    node.maintenance_command = {}
    node.maintenance_deadline = 0.0
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        node.joints[f"{name}_steer_joint"] = {"position": 0.0}
    return node


def test_feedback_health_requires_all_fresh_finite_positions():
    node = bare_node()
    assert node._steering_feedback_health()["healthy"]
    del node.joints["rear_right_steer_joint"]
    health = node._steering_feedback_health()
    assert not health["healthy"] and "无位置反馈" in health["reasons"][0]
    node = bare_node()
    node.joints["front_left_steer_joint"]["position"] = math.nan
    assert "非有限值" in node._steering_feedback_health()["reasons"][0]


def test_feedback_health_rejects_timeout_and_recent_jump():
    node = bare_node()
    node.last_joint_time = time.monotonic() - 1.0
    assert "超时" in node._steering_feedback_health()["reasons"][0]
    node = bare_node()
    node.steering_jump_faults["front_left_steer_joint"] = time.monotonic()
    assert any("跳变" in reason for reason in node._steering_feedback_health()["reasons"])


def test_maintenance_mode_switches_controllers_and_verifies_state():
    node = bare_node()
    node.mode = "DISABLED"
    calls = []
    node.stop = lambda reason: calls.append(("stop", reason))
    node._switch_controllers = lambda active, inactive: calls.append((active, inactive))
    snapshots = iter([
        {"swerve_controller": "active", "steering_maintenance_controller": "inactive",
         "drive_maintenance_controller": "inactive"},
        {"swerve_controller": "inactive", "steering_maintenance_controller": "active",
         "drive_maintenance_controller": "active"},
    ])
    node._list_controllers = lambda: next(snapshots)
    node._event = lambda level, message: None
    result = node.set_mode({
        "mode": "MAINTENANCE", "acknowledge_wheels_lifted": True
    })
    assert result["mode"] == "MAINTENANCE"
    assert calls[1] == (["steering_maintenance_controller", "drive_maintenance_controller"],
                        ["swerve_controller"])


def test_failed_switch_falls_back_to_disabled():
    node = bare_node()
    node.mode = "NORMAL"
    node.stop = lambda reason: None
    node._list_controllers = lambda: {
        "swerve_controller": "active", "steering_maintenance_controller": "inactive",
        "drive_maintenance_controller": "inactive",
    }
    node._switch_controllers = lambda *args: (_ for _ in ()).throw(RuntimeError("switch failed"))
    node._publish_maintenance_stop = lambda: None
    with pytest.raises(RuntimeError, match="switch failed"):
        node.set_mode({
            "mode": "MAINTENANCE", "acknowledge_wheels_lifted": True
        })
    assert node.mode == "DISABLED"


def test_maintenance_clamps_more_than_configured_step():
    node = bare_node()
    node.mode = "MAINTENANCE"
    result = node.maintenance_command_request({
        "deadman": True, "wheel": "front_left", "rpm": 0.0,
        "relative_angle_deg": 180.2, "command_id": "too-large",
    })
    assert result["command"]["angle_deg"] == pytest.approx(90.0)
    assert result["command"]["clamped"]


def test_maintenance_stop_holds_current_steering(monkeypatch):
    node = bare_node()
    node.maintenance_command = {"wheel": "front_left", "rpm": 3.0, "angle_deg": 2.0}
    node.joints["front_left_steer_joint"]["position"] = 1.25

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(list(message.data))

    node.steering_maintenance_publisher = Publisher()
    node.drive_maintenance_publisher = Publisher()
    monkeypatch.setattr("mobile_base_web_console.web_console.rclpy.ok", lambda: True)
    node._publish_maintenance_stop()
    assert node.steering_maintenance_publisher.messages[-1][0] == pytest.approx(1.25)
    assert node.drive_maintenance_publisher.messages[-1] == [0.0] * 4



def test_maintenance_relative_target_is_latched_once(monkeypatch):
    node = bare_node()
    node.mode = "MAINTENANCE"
    node.maintenance_command = {}
    node.joints["front_left_steer_joint"]["position"] = math.radians(80.0)
    monkeypatch.setattr("mobile_base_web_console.web_console.rclpy.ok", lambda: False)
    body = {
        "deadman": True, "wheel": "front_left",
        "rpm": 0.0, "relative_angle_deg": 3.0, "command_id": "one-press",
    }
    first = node.maintenance_command_request(body)
    assert first["command"]["angle_deg"] == pytest.approx(83.0)
    node.joints["front_left_steer_joint"]["position"] = math.radians(82.0)
    second = node.maintenance_command_request(body)
    assert second["command"]["angle_deg"] == pytest.approx(83.0)


def test_maintenance_clamps_target_to_mechanical_limit():
    node = bare_node()
    node.mode = "MAINTENANCE"
    node.maintenance_command = {}
    node.joints["front_left_steer_joint"]["position"] = math.radians(80.0)
    result = node.maintenance_command_request({
        "deadman": True, "wheel": "front_left", "rpm": 0.0,
        "relative_angle_deg": 11.0, "command_id": "beyond-limit",
    })
    assert result["command"]["angle_deg"] == pytest.approx(90.0)
    assert result["command"]["clamped"]


def test_maintenance_supports_absolute_target_and_clamps_it():
    node = bare_node()
    node.mode = "MAINTENANCE"
    node.maintenance_command = {}
    result = node.maintenance_command_request({
        "deadman": True, "wheel": "rear_right", "rpm": 0.0,
        "angle_mode": "absolute", "target_angle_deg": -120.0,
        "command_id": "absolute-limit",
    })
    assert result["command"]["angle_deg"] == pytest.approx(-90.0)
    assert result["command"]["clamped"]
