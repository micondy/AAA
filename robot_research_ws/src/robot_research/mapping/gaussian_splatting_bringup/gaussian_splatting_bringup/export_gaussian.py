from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a trained Nerfstudio Gaussian model to PLY.")
    parser.add_argument("config", type=Path, help="Nerfstudio config.yml produced by ns-train")
    parser.add_argument("--output-dir", type=Path, default=Path("exports/gaussian_splat"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    config = args.config.expanduser().resolve()
    if not config.is_file():
        raise SystemExit(f"training config does not exist: {config}")
    executable = shutil.which("ns-export")
    if executable is None and not args.dry_run:
        raise SystemExit("ns-export is not installed; activate the dedicated Nerfstudio environment")
    command = [
        executable or "ns-export", "gaussian-splat",
        "--load-config", str(config),
        "--output-dir", str(args.output_dir.expanduser().resolve()),
    ]
    print(" ".join(command), flush=True)
    if not args.dry_run:
        subprocess.run(command, check=True)
