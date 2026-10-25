#!/usr/bin/env python3
"""Convert the exported greenhouse STL to a cached USD asset for Isaac Sim."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
import sys

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_known_args()[0]


arguments = parse_args()
application = SimulationApp({"headless": True})

import omni.kit.asset_converter
from isaacsim.core.experimental.utils.app import enable_extension


async def convert(source: Path, destination: Path) -> bool:
    """Convert one STL into a single-mesh USD while preserving STL units."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = True
    context.single_mesh = True
    context.smooth_normals = True
    context.use_meter_as_world_unit = False

    def progress_callback(progress: float, total_steps: int) -> None:
        if total_steps:
            print(f"[rack_stl_converter] {100.0 * progress / total_steps:.1f}%", flush=True)

    task = omni.kit.asset_converter.get_instance().create_converter_task(
        str(source.resolve()),
        str(destination.resolve()),
        progress_callback,
        context,
    )
    success = await task.wait_until_finished()
    if not success:
        print(
            f"conversion failed: status={task.get_status()} error={task.get_error_message()}",
            file=sys.stderr,
        )
    return bool(success)


try:
    if not arguments.input.is_file():
        raise FileNotFoundError(arguments.input)
    enable_extension("omni.kit.asset_converter")
    application.update()
    conversion_future = asyncio.ensure_future(
        convert(arguments.input, arguments.output)
    )
    while not conversion_future.done():
        application.update()
    converted = conversion_future.result()
finally:
    application.close()

raise SystemExit(0 if converted else 1)
