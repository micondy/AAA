"""Pure helpers for repeatable target-perception benchmark artifacts."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import fmean, pstdev


CSV_FIELDS = (
    "sample_index",
    "stamp_sec",
    "stamp_nanosec",
    "detected",
    "frame_id",
    "x_m",
    "y_m",
    "z_m",
    "error_x_m",
    "error_y_m",
    "error_z_m",
    "position_error_m",
    "size_x_m",
    "size_y_m",
    "size_error_x_m",
    "size_error_y_m",
    "confidence",
    "valid_depth_points",
    "valid_depth_ratio",
)


def _stat(values: list[float], operation) -> float | None:
    return operation(values) if values else None


def summarize_rows(rows: list[dict[str, object]]) -> dict[str, object]:
    """Return detection, position, size, and depth repeatability statistics."""
    detected = [row for row in rows if bool(row["detected"])]

    def values(name: str) -> list[float]:
        return [float(row[name]) for row in detected if row.get(name) is not None]

    errors = values("position_error_m")
    summary: dict[str, object] = {
        "samples": len(rows),
        "detections": len(detected),
        "failures": len(rows) - len(detected),
        "detection_rate": len(detected) / len(rows) if rows else 0.0,
        "position_error_mean_m": _stat(errors, fmean),
        "position_error_std_m": _stat(errors, pstdev),
        "position_error_rmse_m": (
            math.sqrt(fmean([value * value for value in errors])) if errors else None
        ),
        "position_error_max_m": _stat(errors, max),
        "bias_x_m": _stat(values("error_x_m"), fmean),
        "bias_y_m": _stat(values("error_y_m"), fmean),
        "bias_z_m": _stat(values("error_z_m"), fmean),
        "size_error_x_mean_m": _stat(values("size_error_x_m"), fmean),
        "size_error_y_mean_m": _stat(values("size_error_y_m"), fmean),
        "valid_depth_ratio_mean": _stat(values("valid_depth_ratio"), fmean),
        "confidence_mean": _stat(values("confidence"), fmean),
    }
    return summary


def write_artifacts(
    rows: list[dict[str, object]],
    csv_path: str | Path,
    metadata: dict[str, object],
) -> tuple[Path, Path, dict[str, object]]:
    """Write per-frame CSV and an adjacent JSON summary."""
    output = Path(csv_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    summary = {"metadata": metadata, "metrics": summarize_rows(rows)}
    summary_path = output.with_suffix(".summary.json")
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return output, summary_path, summary
