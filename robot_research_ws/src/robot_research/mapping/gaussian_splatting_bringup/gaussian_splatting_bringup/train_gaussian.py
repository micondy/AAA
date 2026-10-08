from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess


def main() -> None:
    parser = argparse.ArgumentParser(description="Train Nerfstudio Splatfacto on a captured dataset.")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/gaussian_splatting"))
    parser.add_argument("--method", choices=("splatfacto", "splatfacto-big"), default="splatfacto")
    parser.add_argument("--max-num-iterations", type=int, default=30000)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    transforms = args.dataset.expanduser().resolve() / "transforms.json"
    if not transforms.is_file():
        raise SystemExit(f"dataset is missing transforms.json: {transforms}")
    metadata = json.loads(transforms.read_text(encoding="utf-8"))
    if len(metadata.get("frames", [])) < 20:
        raise SystemExit("at least 20 captured frames are required")
    executable = shutil.which("ns-train")
    if executable is None and not args.dry_run:
        raise SystemExit("ns-train is not installed; activate the dedicated Nerfstudio environment")
    command = [
        executable or "ns-train", args.method,
        "--max-num-iterations", str(args.max_num_iterations),
        "--output-dir", str(args.output_dir.expanduser().resolve()),
        "--data", str(args.dataset.expanduser().resolve()),
    ]
    print(" ".join(command), flush=True)
    if not args.dry_run:
        subprocess.run(command, check=True)
