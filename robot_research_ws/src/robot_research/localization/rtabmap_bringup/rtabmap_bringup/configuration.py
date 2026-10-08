from pathlib import Path
import re

import yaml


VALID_MODES = ("mapping", "localization")
ROS_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*$")


def as_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def camera_prefix(camera_namespace: str, camera_name: str) -> str:
    namespace = camera_namespace.strip().strip("/")
    name = camera_name.strip().strip("/")
    if not namespace or not name:
        raise ValueError("camera_namespace and camera_name must not be empty")
    if not ROS_NAME_PATTERN.fullmatch(namespace):
        raise ValueError(f"invalid camera_namespace: {camera_namespace!r}")
    if not ROS_NAME_PATTERN.fullmatch(name) or "/" in name:
        raise ValueError(f"invalid camera_name: {camera_name!r}")
    return f"/{namespace}/{name}"


def load_flat_yaml(path: str) -> dict:
    config_path = Path(path).expanduser()
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"configuration is not a dictionary: {config_path}")
    return document


def mode_parameters(mode: str) -> dict[str, str]:
    normalized = mode.strip().lower()
    if normalized not in VALID_MODES:
        raise ValueError(f"mode must be one of: {', '.join(VALID_MODES)}")
    return {
        "Mem/IncrementalMemory": "true" if normalized == "mapping" else "false",
        "Mem/InitWMWithAllNodes": "false" if normalized == "mapping" else "true",
    }
