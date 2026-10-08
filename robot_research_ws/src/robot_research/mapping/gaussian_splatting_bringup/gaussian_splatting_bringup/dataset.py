from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np


OPTICAL_TO_OPENGL = np.diag([1.0, -1.0, -1.0, 1.0])


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


def transform_matrix(translation, rotation) -> np.ndarray:
    matrix = quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def nerfstudio_camera_to_world(ros_optical_to_world: np.ndarray) -> np.ndarray:
    """Convert ROS optical axes (+x right,+y down,+z forward) to OpenGL camera axes."""
    matrix = np.asarray(ros_optical_to_world, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("camera transform must be 4x4")
    return matrix @ OPTICAL_TO_OPENGL


def rotation_angle(first: np.ndarray, second: np.ndarray) -> float:
    relative = first[:3, :3].T @ second[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(cosine))


def should_capture(
    previous: np.ndarray | None,
    current: np.ndarray,
    previous_time_s: float | None,
    current_time_s: float,
    minimum_interval_s: float,
    minimum_translation_m: float,
    minimum_rotation_rad: float,
) -> bool:
    if previous_time_s is not None and current_time_s - previous_time_s < minimum_interval_s:
        return False
    if previous is None:
        return True
    translation = float(np.linalg.norm(current[:3, 3] - previous[:3, 3]))
    rotation = rotation_angle(previous, current)
    return translation >= minimum_translation_m or rotation >= minimum_rotation_rad


def depth_to_millimeters(depth: np.ndarray, encoding: str, integer_scale_m: float) -> np.ndarray:
    array = np.asarray(depth)
    if encoding in ("32FC1", "64FC1"):
        meters = array.astype(np.float64)
    elif encoding in ("16UC1", "mono16"):
        meters = array.astype(np.float64) * integer_scale_m
    else:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    valid = np.isfinite(meters) & (meters > 0.0) & (meters < 65.535)
    result = np.zeros(meters.shape, dtype=np.uint16)
    result[valid] = np.rint(meters[valid] * 1000.0).astype(np.uint16)
    return result


def depth_points_world(
    depth_mm: np.ndarray,
    bgr: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    optical_to_world: np.ndarray,
    stride: int,
    maximum_depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    fx, fy, cx, cy = intrinsics
    rows, columns = np.mgrid[0 : depth_mm.shape[0] : stride, 0 : depth_mm.shape[1] : stride]
    depth = depth_mm[::stride, ::stride].astype(np.float32) * 0.001
    valid = (depth > 0.0) & (depth <= maximum_depth_m)
    z = depth[valid]
    x = (columns[valid].astype(np.float32) - cx) * z / fx
    y = (rows[valid].astype(np.float32) - cy) * z / fy
    homogeneous = np.column_stack((x, y, z, np.ones_like(z)))
    points = (optical_to_world @ homogeneous.T).T[:, :3].astype(np.float32)
    colors = bgr[::stride, ::stride][valid][:, ::-1].astype(np.uint8)
    return points, colors


def write_binary_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    if len(points) != len(colors):
        raise ValueError("point/color count mismatch")
    vertex = np.empty(
        len(points),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")],
    )
    vertex["x"], vertex["y"], vertex["z"] = points.T
    vertex["red"], vertex["green"], vertex["blue"] = colors.T
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(vertex)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        vertex.tofile(stream)


def write_json_atomic(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2), encoding="utf-8")
    temporary.replace(path)


def concatenate_seed_points(
    chunks: Iterable[tuple[np.ndarray, np.ndarray]], maximum_points: int
) -> tuple[np.ndarray, np.ndarray]:
    chunks = list(chunks)
    if not chunks:
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
    points = np.concatenate([chunk[0] for chunk in chunks])
    colors = np.concatenate([chunk[1] for chunk in chunks])
    if len(points) > maximum_points:
        indices = np.linspace(0, len(points) - 1, maximum_points, dtype=np.int64)
        points, colors = points[indices], colors[indices]
    return points, colors
