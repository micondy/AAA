#!/usr/bin/env python3
"""Run the thin Isaac profile at several known target front-surface depths."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import os
from pathlib import Path
import shutil
import subprocess


WORKSPACE = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--distances",
        type=float,
        nargs="+",
        default=(0.4, 0.6, 0.8, 1.0),
        help="Camera-to-visible-front-surface distances in meters.",
    )
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--stage-seconds", type=float, default=30.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="New result directory; defaults to a timestamped workspace path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.samples <= 0 or args.warmup_frames < 0 or args.stage_seconds <= 0.0:
        raise SystemExit(
            "samples and stage-seconds must be positive; warmup-frames must be non-negative"
        )
    if any(not 0.2 <= distance <= 3.0 for distance in args.distances):
        raise SystemExit("every distance must be in [0.2, 3.0] m")
    if shutil.which("ros2") is None:
        raise SystemExit(
            "ros2 is unavailable; first source tools/source_robot_stack.bash"
        )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else WORKSPACE / "results" / "target_perception" / f"distance_{stamp}"
    )
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    environment = os.environ.copy()
    environment.setdefault("ROS_HOME", "/tmp/target_distance_benchmark_ros_home")
    environment.setdefault("ROS_LOG_DIR", "/tmp/target_distance_benchmark_ros_logs")
    distance_text = ",".join(str(value) for value in args.distances)
    command = [
        "ros2",
        "launch",
        "mobile_manipulator_bringup",
        "sim_perception.launch.py",
        "headless:=true",
        "run_distance_benchmark:=true",
        f"benchmark_distances:={distance_text}",
        f"benchmark_stage_seconds:={args.stage_seconds}",
        f"benchmark_samples:={args.samples}",
        f"benchmark_warmup_frames:={args.warmup_frames}",
        f"benchmark_output_dir:={output_dir}",
    ]
    print(
        f"[distance_benchmark] one Isaac process; distances={distance_text} m",
        flush=True,
    )
    subprocess.run(command, cwd=WORKSPACE, env=environment, check=True)
    summary_csv = output_dir / "distance_summary.csv"
    if not summary_csv.is_file():
        raise RuntimeError(f"benchmark exited without summary: {summary_csv}")
    with summary_csv.open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    counts = [int(row["samples"]) for row in rows]
    if any(count != args.samples for count in counts):
        raise RuntimeError(
            f"incomplete synchronized samples {counts}; increase --stage-seconds"
        )
    print(f"[distance_benchmark] COMPLETE summary={summary_csv}", flush=True)


if __name__ == "__main__":
    main()
