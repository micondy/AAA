"""Load and validate simple box-based MoveIt task environments."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class SceneBox:
    """One axis-aligned or quaternion-oriented collision box."""

    object_id: str
    size: tuple[float, float, float]
    pose: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class TaskScene:
    """Validated task-scene description."""

    frame_id: str
    objects: tuple[SceneBox, ...]


def _numbers(value, count: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{field} must contain exactly {count} numbers")
    try:
        numbers = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field} contains a non-numeric value") from error
    if not all(math.isfinite(item) for item in numbers):
        raise ValueError(f"{field} contains a non-finite value")
    return numbers


def load_task_scene(path: str | Path) -> TaskScene:
    """Load a YAML scene and reject ambiguous or unsafe geometry."""
    scene_path = Path(path).expanduser().resolve()
    with scene_path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("task scene root must be a mapping")
    frame_id = str(document.get("frame_id", "")).strip()
    if not frame_id:
        raise ValueError("task scene frame_id must not be empty")
    raw_objects = document.get("objects")
    if not isinstance(raw_objects, list) or not raw_objects:
        raise ValueError("task scene must contain at least one object")

    boxes = []
    identifiers = set()
    for index, item in enumerate(raw_objects):
        if not isinstance(item, dict):
            raise ValueError(f"objects[{index}] must be a mapping")
        object_id = str(item.get("id", "")).strip()
        if not object_id or object_id in identifiers:
            raise ValueError(f"objects[{index}] has an empty or duplicate id")
        identifiers.add(object_id)
        size = _numbers(item.get("size"), 3, f"objects[{index}].size")
        if any(value <= 0.0 for value in size):
            raise ValueError(f"objects[{index}].size must be positive")
        pose = _numbers(item.get("pose"), 7, f"objects[{index}].pose")
        quaternion_norm = math.sqrt(sum(value * value for value in pose[3:]))
        if not math.isclose(quaternion_norm, 1.0, abs_tol=1.0e-3):
            raise ValueError(f"objects[{index}].pose quaternion must be normalized")
        boxes.append(SceneBox(object_id=object_id, size=size, pose=pose))
    return TaskScene(frame_id=frame_id, objects=tuple(boxes))
