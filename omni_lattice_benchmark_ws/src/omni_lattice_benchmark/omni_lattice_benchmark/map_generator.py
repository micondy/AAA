#!/usr/bin/env python3
"""Generate deterministic 0.05 m occupancy maps for planner comparisons."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path


FREE = 254
OCCUPIED = 0


@dataclass
class Grid:
    width: int = 200
    height: int = 200
    resolution: float = 0.05

    def __post_init__(self) -> None:
        self.cells = [FREE] * (self.width * self.height)

    def fill(self, value: int) -> None:
        self.cells[:] = [value] * len(self.cells)

    def set_cell(self, x: int, y: int, value: int) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            self.cells[y * self.width + x] = value

    def rectangle(self, x0: float, y0: float, x1: float, y1: float, value: int) -> None:
        ix0 = max(0, int(round(x0 / self.resolution)))
        iy0 = max(0, int(round(y0 / self.resolution)))
        ix1 = min(self.width, int(round(x1 / self.resolution)))
        iy1 = min(self.height, int(round(y1 / self.resolution)))
        for y in range(iy0, iy1):
            offset = y * self.width
            for x in range(ix0, ix1):
                self.cells[offset + x] = value

    def border(self, thickness: float = 0.20) -> None:
        t = int(round(thickness / self.resolution))
        for y in range(self.height):
            for x in range(self.width):
                if x < t or y < t or x >= self.width - t or y >= self.height - t:
                    self.set_cell(x, y, OCCUPIED)

    def write(self, output: Path, name: str) -> None:
        output.mkdir(parents=True, exist_ok=True)
        pgm = output / f"{name}.pgm"
        # PGM rows are top-to-bottom while OccupancyGrid world y grows bottom-to-top.
        payload = bytearray()
        for y in range(self.height - 1, -1, -1):
            start = y * self.width
            payload.extend(self.cells[start : start + self.width])
        pgm.write_bytes(
            f"P5\n{self.width} {self.height}\n255\n".encode("ascii") + payload
        )
        (output / f"{name}.yaml").write_text(
            "\n".join(
                [
                    f"image: {name}.pgm",
                    f"resolution: {self.resolution}",
                    "origin: [0.0, 0.0, 0.0]",
                    "negate: 0",
                    "occupied_thresh: 0.65",
                    "free_thresh: 0.196",
                    "mode: trinary",
                    "",
                ]
            ),
            encoding="utf-8",
        )


def open_space() -> Grid:
    grid = Grid()
    grid.border()
    # One isolated obstacle prevents this case from degenerating into a trivial line test.
    grid.rectangle(4.35, 4.35, 5.65, 5.65, OCCUPIED)
    return grid


def l_corridor() -> Grid:
    grid = Grid()
    grid.fill(OCCUPIED)
    # A 1.05 m-wide L corridor. The 0.66 x 0.62 m navigation footprint fits,
    # but its heading must change around the inside corner.
    grid.rectangle(0.35, 1.00, 6.05, 2.05, FREE)
    grid.rectangle(5.00, 1.00, 6.05, 9.65, FREE)
    return grid


def dense_obstacles() -> Grid:
    grid = Grid()
    grid.border()
    obstacles = [
        (1.6, 1.2, 2.5, 4.2),
        (1.2, 6.0, 3.2, 7.0),
        (3.6, 2.5, 4.6, 8.0),
        (5.4, 0.8, 6.4, 5.3),
        (5.6, 7.0, 7.7, 8.0),
        (7.4, 2.5, 8.5, 6.6),
    ]
    for obstacle in obstacles:
        grid.rectangle(*obstacle, OCCUPIED)
    return grid


def narrow_irregular() -> Grid:
    grid = Grid()
    grid.border()
    # Staggered walls produce two narrow doors and a final offset passage.
    grid.rectangle(2.7, 0.2, 3.25, 3.55, OCCUPIED)
    grid.rectangle(2.7, 4.55, 3.25, 9.8, OCCUPIED)
    grid.rectangle(5.3, 0.2, 5.85, 5.15, OCCUPIED)
    grid.rectangle(5.3, 6.15, 5.85, 9.8, OCCUPIED)
    grid.rectangle(7.5, 0.2, 8.05, 2.65, OCCUPIED)
    grid.rectangle(7.5, 3.65, 8.05, 9.8, OCCUPIED)
    # The three openings occur at different heights, so the path must combine
    # longitudinal and lateral motion even without extra interior blocks.
    return grid


GENERATORS = {
    "open_space": open_space,
    "l_corridor": l_corridor,
    "dense_obstacles": dense_obstacles,
    "narrow_irregular": narrow_irregular,
}


def generate_all(output: Path) -> None:
    for name, generator in GENERATORS.items():
        generator().write(output, name)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("maps"))
    args = parser.parse_args()
    generate_all(args.output)
    print(f"Generated {len(GENERATORS)} maps in {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
