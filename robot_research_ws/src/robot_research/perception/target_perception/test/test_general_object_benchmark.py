import json

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from target_perception.general_object_benchmark import (
    summarize_general_objects,
    write_general_object_artifacts,
)
from target_perception.general_object_benchmark_node import _diagnostic


def _frames():
    return [
        {
            "frame_index": 0,
            "stamp_sec": 1,
            "stamp_nanosec": 0,
            "frame_id": "camera",
            "detection_count": 2,
            "model_instance_count": 3,
            "depth_valid_instance_count": 2,
            "depth_valid_ratio": 2 / 3,
            "inference_and_fusion_ms": 40.0,
            "device": "cuda",
            "weights_sha256": "abc",
            "diagnostic_level": 0,
            "diagnostic_message": "instances detected",
        },
        {
            "frame_index": 1,
            "stamp_sec": 2,
            "stamp_nanosec": 0,
            "frame_id": "camera",
            "detection_count": 0,
            "model_instance_count": 1,
            "depth_valid_instance_count": 0,
            "depth_valid_ratio": 0.0,
            "inference_and_fusion_ms": 60.0,
            "device": "cuda",
            "weights_sha256": "abc",
            "diagnostic_level": 1,
            "diagnostic_message": "no depth-valid instances",
        },
    ]


def _instances():
    common = {
        "frame_index": 0,
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 1.0,
        "size_x_m": 0.2,
        "size_y_m": 0.2,
        "size_z_m": 0.1,
        "position_covariance_trace_m2": 0.001,
    }
    return [
        {
            **common,
            "instance_index": 0,
            "detection_id": "person_01",
            "class_id": "person",
            "confidence": 0.9,
        },
        {
            **common,
            "instance_index": 1,
            "detection_id": "cup_02",
            "class_id": "cup",
            "confidence": 0.7,
        },
    ]


def test_general_summary_counts_frames_classes_depth_and_latency():
    summary = summarize_general_objects(_frames(), _instances())

    assert summary["samples"] == 2
    assert summary["frame_detection_rate"] == 0.5
    assert summary["instances_per_frame_mean"] == 1.0
    assert summary["depth_valid_instance_ratio"] == 0.5
    assert summary["confidence_mean"] == pytest.approx(0.8)
    assert summary["latency_mean_ms"] == 50.0
    assert summary["latency_p95_ms"] == 60.0
    assert summary["class_counts"] == {"cup": 1, "person": 1}


def test_general_artifacts_are_machine_readable_and_refuse_overwrite(tmp_path):
    output = tmp_path / "run_001"
    frame_path, instance_path, summary_path, artifact = (
        write_general_object_artifacts(
            _frames(), _instances(), output, {"weights_sha256": ["abc"]}
        )
    )

    assert frame_path.is_file()
    assert instance_path.is_file()
    assert json.loads(summary_path.read_text(encoding="utf-8")) == artifact
    with pytest.raises(FileExistsError, match="not empty"):
        write_general_object_artifacts(_frames(), _instances(), output, {})


def test_diagnostic_level_accepts_ros_byte_representation():
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.level = DiagnosticStatus.WARN
    status.message = "warning"
    message.status.append(status)

    level, text, values = _diagnostic(message)
    assert level == 1
    assert text == "warning"
    assert values == {}
