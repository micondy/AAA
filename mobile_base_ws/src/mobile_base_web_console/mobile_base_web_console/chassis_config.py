from __future__ import annotations

import copy
import datetime
import os
import re
import threading
from pathlib import Path
from typing import Any

import yaml

WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")


def _integer(value: Any, name: str, minimum: int, maximum: int) -> int:
    if isinstance(value, str):
        result = int(value.strip(), 0)
    else:
        result = int(value)
    if not minimum <= result <= maximum:
        raise ValueError(f"{name} 必须在 {minimum} 到 {maximum} 之间")
    return result


def _positive(value: Any, name: str, maximum: float) -> float:
    result = float(value)
    if not 0.0 < result <= maximum:
        raise ValueError(f"{name} 必须大于 0 且不超过 {maximum}")
    return result


def _direction(value: Any, name: str) -> int:
    result = int(value)
    if result not in (-1, 1):
        raise ValueError(f"{name} 只能是 1 或 -1")
    return result


class ChassisConfigManager:
    """底盘硬件的单一配置入口，负责严格校验、原子保存和历史备份。"""

    def __init__(self, default_path: Path):
        config_dir = Path(
            os.environ.get("MOBILE_BASE_CONFIG_DIR", Path.home() / ".config" / "mobile_base")
        ).expanduser()
        self.path = config_dir / "chassis.yaml"
        self.backup_dir = config_dir / "backup"
        self._lock = threading.RLock()
        source = self.path if self.path.exists() else default_path
        self._config = self.validate(yaml.safe_load(source.read_text(encoding="utf-8")) or {})

    def get(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._config)

    def save(self, candidate: dict[str, Any]) -> dict[str, Any]:
        validated = self.validate(copy.deepcopy(candidate))
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            backup = None
            if self.path.exists():
                self.backup_dir.mkdir(parents=True, exist_ok=True)
                stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
                backup = self.backup_dir / f"chassis-{stamp}.yaml"
                backup.write_bytes(self.path.read_bytes())
            temporary = self.path.with_suffix(".yaml.tmp")
            temporary.write_text(
                yaml.safe_dump(validated, allow_unicode=True, sort_keys=False), encoding="utf-8"
            )
            temporary.replace(self.path)
            self._config = validated
            return {"config": copy.deepcopy(validated), "backup": str(backup) if backup else None}

    @staticmethod
    def validate(config: dict[str, Any]) -> dict[str, Any]:
        if int(config.get("schema_version", 0)) != 1:
            raise ValueError("不支持的 chassis.yaml 版本")
        for section in ("can", "geometry", "safety", "wheels"):
            if not isinstance(config.get(section), dict):
                raise ValueError(f"缺少配置对象：{section}")

        for bus in ("steering", "drive"):
            item = config["can"].get(bus)
            if not isinstance(item, dict):
                raise ValueError(f"缺少 {bus} CAN 配置")
            interface = str(item.get("interface", "")).strip()
            if not re.fullmatch(r"[A-Za-z0-9_.:-]{1,32}", interface):
                raise ValueError(f"{bus} CAN 接口名称无效")
            item["interface"] = interface
            item["bitrate"] = _integer(item.get("bitrate"), f"{bus} bitrate", 10000, 10000000)
            item["fd"] = bool(item.get("fd", False))

        geometry = config["geometry"]
        geometry["wheel_radius"] = _positive(geometry.get("wheel_radius"), "轮径", 2.0)
        geometry["wheelbase"] = _positive(geometry.get("wheelbase"), "轴距", 10.0)
        geometry["track_width"] = _positive(geometry.get("track_width"), "轮距", 10.0)
        geometry["wheel_width"] = _positive(
            geometry.get("wheel_width", 0.0683), "轮胎宽度", 1.0
        )
        geometry["wheel_center_offset"] = _positive(
            geometry.get("wheel_center_offset", 0.043), "转向轴到轮心偏置", 1.0
        )
        # CAD envelope used by the map and nvblox overlays. Defaults keep
        # existing saved configurations compatible.
        geometry["body_length"] = _positive(
            geometry.get("body_length", 0.62), "车体长度", 10.0
        )
        geometry["body_width"] = _positive(
            geometry.get("body_width", 0.58), "车体宽度", 10.0
        )
        geometry["footprint_padding"] = _positive(
            geometry.get("footprint_padding", 0.02), "导航轮廓余量", 1.0
        )

        safety = config["safety"]
        safety["max_wheel_angular_velocity"] = _positive(
            safety.get("max_wheel_angular_velocity", 20.0), "整车最大轮角速度", 200.0
        )
        safety["max_wheel_angular_acceleration"] = _positive(
            safety.get("max_wheel_angular_acceleration", 8.0), "整车最大轮角加速度", 500.0
        )
        safety["steering_limit_deg"] = _positive(
            safety.get("steering_limit_deg", 90.0), "转向机械单侧限位", 90.0
        )
        # 旧版边界夹角参数已淘汰；不再以夹角继续行走。
        safety.pop("steering_flip_hysteresis_deg", None)
        safety["drive_alignment_tolerance_deg"] = _positive(
            safety.get("drive_alignment_tolerance_deg", 5.0), "行走放行转向误差", 45.0
        )
        safety["controller_command_timeout"] = _positive(
            safety.get("controller_command_timeout", 0.25), "整车控制超时", 5.0
        )
        safety["steering_feedback_timeout"] = _positive(
            safety.get("steering_feedback_timeout", 0.5), "转向反馈超时", 5.0
        )
        safety["steering_max_feedback_jump_rad"] = _positive(
            safety.get("steering_max_feedback_jump_rad", 1.0), "转向反馈跳变阈值", 6.3
        )
        safety["maintenance_max_rpm"] = _positive(
            safety.get("maintenance_max_rpm"), "维护最大转速", 200.0
        )
        safety["maintenance_max_steering_angle_deg"] = _positive(
            safety.get("maintenance_max_steering_angle_deg"), "维护最大转角", 360.0
        )
        safety["maintenance_max_steering_step_deg"] = _positive(
            safety.get("maintenance_max_steering_step_deg", 5.0), "维护单次最大转角", 360.0
        )
        if safety["maintenance_max_steering_angle_deg"] > safety["steering_limit_deg"]:
            raise ValueError("维护最大转角不能超过转向机械单侧限位")
        safety["maintenance_max_steering_speed_rad_s"] = _positive(
            safety.get("maintenance_max_steering_speed_rad_s"), "维护转向速度", 10.0
        )
        safety["command_timeout"] = _positive(
            safety.get("command_timeout"), "维护心跳超时", 5.0
        )
        if safety["command_timeout"] < 0.2:
            raise ValueError("维护心跳超时不能小于 0.2 秒")

        if set(config["wheels"]) != set(WHEEL_NAMES):
            raise ValueError("wheels 必须且只能包含四个标准轮组")
        motor_ids: set[int] = set()
        master_ids: set[int] = set()
        drive_channels: set[tuple[int, int]] = set()
        for wheel_name in WHEEL_NAMES:
            wheel = config["wheels"][wheel_name]
            steering, drive = wheel.get("steering"), wheel.get("drive")
            if not isinstance(steering, dict) or not isinstance(drive, dict):
                raise ValueError(f"{wheel_name} 缺少 steering 或 drive")
            wheel["label"] = str(wheel.get("label", wheel_name))[:20]
            steering["motor_id"] = _integer(steering.get("motor_id"), f"{wheel_name} motor_id", 1, 15)
            steering["master_id"] = _integer(steering.get("master_id"), f"{wheel_name} master_id", 0, 2047)
            steering["direction"] = _direction(steering.get("direction"), f"{wheel_name} 转向方向")
            steering["offset_rad"] = float(steering.get("offset_rad", 0.0))
            if not -100.0 <= steering["offset_rad"] <= 100.0:
                raise ValueError(f"{wheel_name} 转向零偏超出范围")
            steering["max_velocity_rad_s"] = _positive(
                steering.get("max_velocity_rad_s"), f"{wheel_name} 转向速度", 30.0
            )
            drive["node_id"] = _integer(drive.get("node_id"), f"{wheel_name} node_id", 1, 127)
            drive["channel"] = _integer(drive.get("channel"), f"{wheel_name} channel", 1, 2)
            drive["direction"] = _direction(drive.get("direction"), f"{wheel_name} 行走方向")
            drive["gear_ratio"] = _positive(drive.get("gear_ratio"), f"{wheel_name} 减速比", 1000.0)
            drive["max_motor_rpm"] = _positive(
                drive.get("max_motor_rpm"), f"{wheel_name} 最大电机转速", 100000.0
            )
            if steering["motor_id"] in motor_ids:
                raise ValueError(f"达妙 motor_id 重复：{steering['motor_id']}")
            if steering["master_id"] in master_ids:
                raise ValueError(f"达妙 master_id 重复：{steering['master_id']}")
            pair = (drive["node_id"], drive["channel"])
            if pair in drive_channels:
                raise ValueError(f"ZLAC 节点/通道重复：{pair[0]}/{pair[1]}")
            motor_ids.add(steering["motor_id"])
            master_ids.add(steering["master_id"])
            drive_channels.add(pair)
        return config
