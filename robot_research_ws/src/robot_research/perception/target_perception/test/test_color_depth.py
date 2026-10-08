import ast
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from target_perception.benchmark import summarize_rows, write_artifacts
from target_perception.color_depth import (
    CameraIntrinsics,
    color_mask,
    depth_to_meters,
    estimate_color_target,
    estimate_mask_target,
)


def _green_scene():
    color = np.zeros((120, 160, 3), dtype=np.uint8)
    color[40:80, 60:100] = (0, 255, 0)
    depth = np.full((120, 160), np.nan, dtype=np.float32)
    depth[40:80, 60:100] = 1.0
    return color, depth


def test_synthetic_green_target_is_deprojected_at_one_meter():
    color, depth = _green_scene()
    depth[50, 70] = 2.5
    estimate = estimate_color_target(
        color,
        depth,
        CameraIntrinsics(fx=100.0, fy=100.0, cx=79.5, cy=59.5),
        morphology_kernel=1,
    )

    assert estimate is not None
    assert estimate.position == pytest.approx((0.0, 0.0, 1.0), abs=0.006)
    assert estimate.size[:2] == pytest.approx((0.40, 0.40), abs=0.011)
    assert estimate.valid_depth_points >= 1500
    assert estimate.valid_depth_ratio > 0.95
    assert 0.9 <= estimate.confidence <= 1.0
    assert estimate.bbox_xywh == (60, 40, 40, 40)


def test_arbitrary_segmentation_mask_reuses_rgbd_geometry():
    depth = np.full((80, 100), 1.25, dtype=np.float32)
    mask = np.zeros((80, 100), dtype=bool)
    mask[20:60, 30:70] = True
    estimate = estimate_mask_target(
        mask,
        depth,
        CameraIntrinsics(fx=100.0, fy=100.0, cx=50.0, cy=40.0),
        min_mask_area=100.0,
        prior_confidence=0.8,
    )

    assert estimate is not None
    assert estimate.bbox_xywh == (30, 20, 40, 40)
    assert estimate.position[2] == pytest.approx(1.25)
    assert estimate.confidence == pytest.approx(0.8)


def test_mask_depth_geometry_rejects_invalid_contracts():
    intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=50.0, cy=40.0)
    with pytest.raises(ValueError, match="dimensions must match"):
        estimate_mask_target(
            np.zeros((10, 10), dtype=np.uint8),
            np.zeros((11, 10), dtype=np.float32),
            intrinsics,
        )
    with pytest.raises(ValueError, match="prior_confidence"):
        estimate_mask_target(
            np.zeros((10, 10), dtype=np.uint8),
            np.zeros((10, 10), dtype=np.float32),
            intrinsics,
            prior_confidence=1.1,
        )


def test_missing_aligned_depth_rejects_color_region():
    color, depth = _green_scene()
    depth[:] = np.nan
    assert estimate_color_target(
        color,
        depth,
        CameraIntrinsics(fx=100.0, fy=100.0, cx=80.0, cy=60.0),
    ) is None


def test_depth_encoding_normalization_and_rejection():
    millimeters = np.asarray([[1000, 1250]], dtype=np.uint16)
    assert depth_to_meters(millimeters, "16UC1", 0.001) == pytest.approx(
        np.asarray([[1.0, 1.25]], dtype=np.float32)
    )
    meters = np.asarray([[1.0]], dtype=np.float32)
    assert depth_to_meters(meters, "32FC1", 0.001) is meters
    with pytest.raises(ValueError, match="unsupported depth encoding"):
        depth_to_meters(millimeters, "8UC1", 0.001)


def test_hue_range_can_wrap_across_red():
    red = np.zeros((20, 20, 3), dtype=np.uint8)
    red[:] = (0, 0, 255)
    mask = color_mask(red, (170, 100, 100), (10, 255, 255), 1)
    assert cv2.countNonZero(mask) == 400


def test_launch_and_default_contract_are_valid():
    package = Path(__file__).parents[1]
    ast.parse((package / "launch" / "color_depth.launch.py").read_text())
    with (package / "config" / "color_depth.yaml").open(encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["color_depth_detector"]["ros__parameters"]
    assert parameters["color_topic"] == "/sensors/nav_camera/color/image_raw"
    assert parameters["depth_topic"].endswith("aligned_depth_to_color/image_raw")
    assert parameters["output_frame"] == "base_link"


def test_benchmark_summary_counts_failures_and_computes_rmse(tmp_path):
    rows = [
        {
            "sample_index": 0,
            "stamp_sec": 1,
            "stamp_nanosec": 0,
            "detected": 1,
            "frame_id": "base_link",
            "x_m": 1.01,
            "y_m": 0.0,
            "z_m": 0.4,
            "error_x_m": 0.01,
            "error_y_m": 0.0,
            "error_z_m": 0.0,
            "position_error_m": 0.01,
            "size_x_m": 0.2,
            "size_y_m": 0.21,
            "size_error_x_m": 0.0,
            "size_error_y_m": 0.01,
            "confidence": 0.9,
            "valid_depth_points": 100,
            "valid_depth_ratio": 0.8,
        },
        {
            "sample_index": 1,
            "stamp_sec": 2,
            "stamp_nanosec": 0,
            "detected": 0,
            "frame_id": "base_link",
            "x_m": None,
            "y_m": None,
            "z_m": None,
            "error_x_m": None,
            "error_y_m": None,
            "error_z_m": None,
            "position_error_m": None,
            "size_x_m": None,
            "size_y_m": None,
            "size_error_x_m": None,
            "size_error_y_m": None,
            "confidence": 0.0,
            "valid_depth_points": 0,
            "valid_depth_ratio": None,
        },
    ]
    summary = summarize_rows(rows)
    assert summary["samples"] == 2
    assert summary["detections"] == 1
    assert summary["detection_rate"] == 0.5
    assert summary["position_error_rmse_m"] == pytest.approx(0.01)

    csv_path, json_path, artifact = write_artifacts(
        rows,
        tmp_path / "benchmark.csv",
        {"truth_front_depth_m": 0.4},
    )
    assert csv_path.is_file()
    assert json_path.is_file()
    assert artifact["metrics"]["failures"] == 1
