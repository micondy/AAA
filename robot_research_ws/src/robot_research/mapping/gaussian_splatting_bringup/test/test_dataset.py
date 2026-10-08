import json
import math
from pathlib import Path

import numpy as np

from gaussian_splatting_bringup.dataset import (
    depth_points_world,
    depth_to_millimeters,
    nerfstudio_camera_to_world,
    should_capture,
    write_binary_ply,
    write_json_atomic,
)


def test_ros_optical_to_nerfstudio_axes():
    converted = nerfstudio_camera_to_world(np.eye(4))
    assert np.allclose(converted[:3, 0], [1, 0, 0])
    assert np.allclose(converted[:3, 1], [0, -1, 0])
    assert np.allclose(converted[:3, 2], [0, 0, -1])


def test_depth_conversion_handles_float_and_invalid_values():
    depth = np.array([[1.25, np.nan, 0.0, 70.0]], np.float32)
    assert depth_to_millimeters(depth, "32FC1", 0.001).tolist() == [[1250, 0, 0, 0]]
    raw = np.array([[500, 1500]], np.uint16)
    assert depth_to_millimeters(raw, "16UC1", 0.001).tolist() == [[500, 1500]]


def test_capture_selection_uses_time_and_motion_thresholds():
    identity = np.eye(4)
    moved = identity.copy()
    moved[0, 3] = 0.04
    assert should_capture(None, identity, None, 1.0, 0.1, 0.03, math.radians(3))
    assert not should_capture(identity, moved, 1.0, 1.05, 0.1, 0.03, math.radians(3))
    assert should_capture(identity, moved, 1.0, 1.2, 0.1, 0.03, math.radians(3))


def test_depth_projection_and_binary_ply(tmp_path: Path):
    depth = np.array([[1000, 0], [0, 2000]], np.uint16)
    bgr = np.array([[[0, 0, 255], [0, 0, 0]], [[0, 0, 0], [255, 0, 0]]], np.uint8)
    points, colors = depth_points_world(depth, bgr, (1, 1, 0, 0), np.eye(4), 1, 3.0)
    assert np.allclose(points, [[0, 0, 1], [2, 2, 2]])
    assert colors.tolist() == [[255, 0, 0], [0, 0, 255]]
    output = tmp_path / "points.ply"
    write_binary_ply(output, points, colors)
    assert output.read_bytes().startswith(b"ply\nformat binary_little_endian 1.0")


def test_atomic_json(tmp_path: Path):
    output = tmp_path / "transforms.json"
    write_json_atomic(output, {"frames": [{"time_seconds": 1.0}]})
    assert json.loads(output.read_text())["frames"][0]["time_seconds"] == 1.0
    assert not output.with_suffix(".json.tmp").exists()
