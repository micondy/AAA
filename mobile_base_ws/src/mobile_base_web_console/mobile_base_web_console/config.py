from __future__ import annotations

import copy
import os
import threading
from pathlib import Path
from typing import Any

import yaml


def _merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


class ConfigManager:
    """加载默认配置，并把用户修改原子写入可迁移的配置目录。"""

    def __init__(self, default_path: Path):
        config_dir = Path(
            os.environ.get("MOBILE_BASE_CONFIG_DIR", Path.home() / ".config" / "mobile_base")
        ).expanduser()
        self.path = config_dir / "web_console.yaml"
        self._lock = threading.RLock()
        self._defaults = yaml.safe_load(default_path.read_text(encoding="utf-8")) or {}
        saved: dict[str, Any] = {}
        if self.path.exists():
            saved = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
        self._config = self._validate(_merge(self._defaults, saved))

    def get(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._config)

    def update(self, update: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        with self._lock:
            previous = self._config
            candidate = self._validate(_merge(previous, update))
            restart_required = any(
                candidate[section] != previous[section] for section in ("web", "topics")
            )
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(".yaml.tmp")
            temporary.write_text(
                yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False),
                encoding="utf-8",
            )
            temporary.replace(self.path)
            self._config = candidate
            return copy.deepcopy(candidate), restart_required

    @staticmethod
    def _validate(config: dict[str, Any]) -> dict[str, Any]:
        required = ("web", "topics", "safety")
        if any(not isinstance(config.get(key), dict) for key in required):
            raise ValueError("配置必须包含 web、topics 和 safety 三个对象")

        config["web"]["host"] = str(config["web"]["host"])
        config["web"]["port"] = int(config["web"]["port"])
        if not 1 <= config["web"]["port"] <= 65535:
            raise ValueError("网页端口必须在 1 到 65535 之间")

        for name in (
            "command", "joint_states", "odometry", "map", "localization_pose",
            "nvblox_mesh",
        ):
            value = str(config["topics"][name]).strip()
            if not value.startswith("/"):
                raise ValueError(f"ROS 话题 {name} 必须以 / 开头")
            config["topics"][name] = value

        for name in ("max_linear_x", "max_linear_y", "max_angular_z"):
            value = float(config["safety"][name])
            if not 0.0 < value <= 20.0:
                raise ValueError(f"安全限值 {name} 超出允许范围")
            config["safety"][name] = value
        return config
