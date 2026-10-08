"""Artifacts and statistics for general multi-object RGB-D perception."""

from __future__ import annotations

from collections import Counter
import csv
import json
import math
from pathlib import Path
from statistics import fmean, pstdev


FRAME_FIELDS = (
    "frame_index",
    "stamp_sec",
    "stamp_nanosec",
    "frame_id",
    "detection_count",
    "model_instance_count",
    "depth_valid_instance_count",
    "depth_valid_ratio",
    "inference_and_fusion_ms",
    "device",
    "weights_sha256",
    "diagnostic_level",
    "diagnostic_message",
)

INSTANCE_FIELDS = (
    "frame_index",
    "instance_index",
    "detection_id",
    "class_id",
    "confidence",
    "x_m",
    "y_m",
    "z_m",
    "size_x_m",
    "size_y_m",
    "size_z_m",
    "position_covariance_trace_m2",
)


def _numbers(rows: list[dict[str, object]], key: str) -> list[float]:
    return [float(row[key]) for row in rows if row.get(key) is not None]


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def summarize_general_objects(
    frame_rows: list[dict[str, object]],
    instance_rows: list[dict[str, object]],
) -> dict[str, object]:
    """Summarize detection availability, throughput, classes, and confidence."""
    frame_count = len(frame_rows)
    detected_frames = sum(int(row["detection_count"]) > 0 for row in frame_rows)
    latencies = _numbers(frame_rows, "inference_and_fusion_ms")
    confidences = _numbers(instance_rows, "confidence")
    model_total = sum(int(row.get("model_instance_count") or 0) for row in frame_rows)
    depth_total = sum(
        int(row.get("depth_valid_instance_count") or 0) for row in frame_rows
    )
    class_counts = Counter(str(row["class_id"]) for row in instance_rows)
    return {
        "samples": frame_count,
        "frames_with_detections": detected_frames,
        "frame_detection_rate": detected_frames / frame_count if frame_count else 0.0,
        "total_depth_valid_instances": len(instance_rows),
        "instances_per_frame_mean": (
            len(instance_rows) / frame_count if frame_count else 0.0
        ),
        "model_instances_total": model_total,
        "depth_valid_instances_total": depth_total,
        "depth_valid_instance_ratio": (
            depth_total / model_total if model_total else None
        ),
        "confidence_mean": fmean(confidences) if confidences else None,
        "latency_mean_ms": fmean(latencies) if latencies else None,
        "latency_std_ms": pstdev(latencies) if latencies else None,
        "latency_p50_ms": _percentile(latencies, 0.50),
        "latency_p95_ms": _percentile(latencies, 0.95),
        "latency_max_ms": max(latencies) if latencies else None,
        "class_counts": dict(sorted(class_counts.items())),
    }


def write_general_object_artifacts(
    frame_rows: list[dict[str, object]],
    instance_rows: list[dict[str, object]],
    output_dir: str | Path,
    metadata: dict[str, object],
) -> tuple[Path, Path, Path, dict[str, object]]:
    """Write frame CSV, instance CSV, and a JSON summary into a fresh directory."""
    output = Path(output_dir).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"benchmark output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    frame_path = output / "frames.csv"
    instance_path = output / "instances.csv"
    summary_path = output / "summary.json"

    for path, fields, rows in (
        (frame_path, FRAME_FIELDS, frame_rows),
        (instance_path, INSTANCE_FIELDS, instance_rows),
    ):
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    artifact = {
        "metadata": metadata,
        "metrics": summarize_general_objects(frame_rows, instance_rows),
    }
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(artifact, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return frame_path, instance_path, summary_path, artifact
