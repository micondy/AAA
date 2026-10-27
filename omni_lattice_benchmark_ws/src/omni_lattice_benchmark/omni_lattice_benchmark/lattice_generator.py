#!/usr/bin/env python3
"""Generate sequential and translation-rotation coupled Omni lattices."""

from __future__ import annotations

import argparse
from datetime import date
import json
import math
from pathlib import Path


# Match the official Nav2 16-heading discretization so the ablation changes the
# control set, not the state discretization.
HEADINGS = [
    0.0,
    math.atan2(1, 2),
    math.pi / 4,
    math.atan2(2, 1),
    math.pi / 2,
    math.pi - math.atan2(2, 1),
    3 * math.pi / 4,
    math.pi - math.atan2(1, 2),
    math.pi,
    math.pi + math.atan2(1, 2),
    5 * math.pi / 4,
    math.pi + math.atan2(2, 1),
    3 * math.pi / 2,
    2 * math.pi - math.atan2(2, 1),
    7 * math.pi / 4,
    2 * math.pi - math.atan2(1, 2),
]

DIRECTIONS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]


def angular_delta(start: float, end: float) -> float:
    return (end - start + math.pi) % (2 * math.pi) - math.pi


def samples(dx: float, dy: float, start: float, end: float, resolution: float) -> list[list[float]]:
    delta = angular_delta(start, end)
    distance = math.hypot(dx, dy)
    count = max(2, math.ceil(distance / (resolution / 2)), math.ceil(abs(delta) / math.radians(5)))
    return [
        [
            round(dx * step / count, 6),
            round(dy * step / count, 6),
            round((start + delta * step / count) % (2 * math.pi), 12),
        ]
        for step in range(1, count + 1)
    ]


def primitive(
    trajectory_id: int,
    start_index: int,
    end_index: int,
    dx: float,
    dy: float,
    resolution: float,
) -> dict:
    start = HEADINGS[start_index]
    end = HEADINGS[end_index]
    delta = angular_delta(start, end)
    length = math.hypot(dx, dy)
    changing_yaw = abs(delta) > 1e-9
    return {
        "trajectory_id": trajectory_id,
        "start_angle_index": start_index,
        "end_angle_index": end_index,
        "left_turn": delta >= 0.0,
        "trajectory_radius": 0.0,
        "trajectory_length": round(length, 6),
        "arc_length": round(length if changing_yaw and length > 0.0 else 0.0, 6),
        "straight_length": round(0.0 if changing_yaw else length, 6),
        "poses": samples(dx, dy, start, end, resolution),
    }


def generate(coupled: bool, resolution: float = 0.05, step_cells: int = 2) -> dict:
    primitives = []
    trajectory_id = 0
    step = resolution * step_cells
    for start_index in range(len(HEADINGS)):
        yaw_offsets = (-1, 0, 1) if coupled else (0,)
        for dx_cell, dy_cell in DIRECTIONS:
            for yaw_offset in yaw_offsets:
                end_index = (start_index + yaw_offset) % len(HEADINGS)
                primitives.append(
                    primitive(
                        trajectory_id,
                        start_index,
                        end_index,
                        dx_cell * step,
                        dy_cell * step,
                        resolution,
                    )
                )
                trajectory_id += 1
        for yaw_offset in (-1, 1):
            end_index = (start_index + yaw_offset) % len(HEADINGS)
            primitives.append(
                primitive(trajectory_id, start_index, end_index, 0.0, 0.0, resolution)
            )
            trajectory_id += 1

    return {
        "version": "1.0",
        "date_generated": date.today().isoformat(),
        "generator": "omni_lattice_benchmark",
        "lattice_metadata": {
            "motion_model": "omni",
            "turning_radius": 0.0,
            "grid_resolution": resolution,
            "stopping_threshold": 0,
            "num_of_headings": len(HEADINGS),
            "heading_angles": HEADINGS,
            "number_of_trajectories": len(primitives),
        },
        "primitives": primitives,
    }


def write_lattices(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for name, coupled in (("omni_sequential_16_5cm.json", False), ("omni_coupled_16_5cm.json", True)):
        document = generate(coupled=coupled)
        (output / name).write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("lattice"))
    args = parser.parse_args()
    write_lattices(args.output)
    print(f"Generated sequential and coupled lattices in {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
