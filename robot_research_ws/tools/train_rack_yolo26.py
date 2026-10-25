#!/usr/bin/env python3
"""Train a two-class rack YOLO26n-seg model from real labeled images."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def check_dataset(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"dataset YAML not found: {path}")
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("dataset YAML must be a mapping")
    names = data.get("names")
    mapping = dict(enumerate(names)) if isinstance(names, list) else names
    if not isinstance(mapping, dict) or {int(k): v for k, v in mapping.items()} != {0: "bed", 1: "post"}:
        raise ValueError("dataset names must be exactly 0: bed, 1: post")
    root = Path(data.get("path", path.parent)).expanduser()
    if not root.is_absolute():
        root = (path.parent / root).resolve()
    for split in ("train", "val"):
        entry = data.get(split)
        if not isinstance(entry, str):
            raise ValueError(f"dataset must provide {split}: images/{split}")
        image_dir = root / entry
        label_dir = root / entry.replace("images", "labels", 1)
        images = [item for item in image_dir.glob("*") if item.suffix.lower() in {".jpg", ".jpeg", ".png"}]
        if not images or not label_dir.is_dir():
            raise ValueError(f"missing {split} images or labels: {image_dir}, {label_dir}")
        label_files = [label_dir / (item.stem + ".txt") for item in images]
        if not any(item.is_file() for item in label_files):
            raise ValueError(f"no polygon label files in {label_dir}")
        valid_polygon = False
        for label_file in label_files:
            if not label_file.is_file():
                continue
            for line in label_file.read_text(encoding="utf-8").splitlines():
                fields = line.split()
                if not fields:
                    continue
                if len(fields) < 7 or len(fields) % 2 != 1 or fields[0] not in {"0", "1"}:
                    raise ValueError(f"invalid YOLO polygon row in {label_file}")
                coordinates = [float(value) for value in fields[1:]]
                if any(not 0.0 <= value <= 1.0 for value in coordinates):
                    raise ValueError(f"polygon coordinates must be normalized in {label_file}")
                valid_polygon = True
        if not valid_polygon:
            raise ValueError(f"no nonempty polygon labels in {label_dir}")
    return data


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True, type=Path, help="YOLO segmentation data.yaml")
    parser.add_argument("--project", default="runs/rack_yolo26")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()
    check_dataset(args.data.resolve())
    from ultralytics import YOLO

    model = YOLO("yolo26n-seg.pt")
    model.train(
        data=str(args.data.resolve()), epochs=args.epochs, imgsz=args.imgsz,
        batch=args.batch, device=args.device, project=str(Path(args.project).resolve()),
        name="bed_post",
    )


if __name__ == "__main__":
    main()
