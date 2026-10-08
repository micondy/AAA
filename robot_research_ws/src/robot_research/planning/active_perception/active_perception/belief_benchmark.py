"""Artifacts and summary metrics for Target Belief convergence experiments."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


FIELDNAMES = (
    "sample",
    "stamp_seconds",
    "x_m",
    "y_m",
    "z_m",
    "position_error_m",
    "variance_x_m2",
    "variance_y_m2",
    "variance_z_m2",
    "covariance_trace_m2",
)


def summarize_rows(rows: list[dict[str, float]]) -> dict[str, float | int]:
    """Summarize convergence without hiding the first and final states."""
    if not rows:
        raise ValueError("at least one belief sample is required")
    initial_trace = rows[0]["covariance_trace_m2"]
    final_trace = rows[-1]["covariance_trace_m2"]
    squared_errors = [row["position_error_m"] ** 2 for row in rows]
    return {
        "sample_count": len(rows),
        "position_error_rmse_m": math.sqrt(sum(squared_errors) / len(rows)),
        "final_position_error_m": rows[-1]["position_error_m"],
        "initial_covariance_trace_m2": initial_trace,
        "final_covariance_trace_m2": final_trace,
        "minimum_covariance_trace_m2": min(
            row["covariance_trace_m2"] for row in rows
        ),
        "covariance_trace_reduction_ratio": (
            1.0 - final_trace / initial_trace if initial_trace > 0.0 else 0.0
        ),
    }


def write_artifacts(
    output_csv: Path,
    rows: list[dict[str, float]],
    metadata: dict[str, object],
) -> dict[str, object]:
    """Write per-sample CSV and a sidecar JSON summary."""
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    artifact = {"metadata": metadata, "metrics": summarize_rows(rows)}
    output_csv.with_suffix(".json").write_text(
        json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return artifact
