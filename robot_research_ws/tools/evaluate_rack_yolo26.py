#!/usr/bin/env python3
"""Evaluate trained rack YOLO26 segmentation weights on a held-out split."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from train_rack_yolo26 import check_dataset


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()
    if not args.weights.is_file():
        parser.error(f"weights not found: {args.weights}")
    check_dataset(args.data.resolve())
    from ultralytics import YOLO

    model = YOLO(str(args.weights.resolve()), task="segment")
    metrics = model.val(
        data=str(args.data.resolve()), split="val", imgsz=args.imgsz,
        batch=args.batch, device=args.device, plots=True,
        project=str((args.output.parent / "yolo26_validation_plots").resolve()),
        name="holdout",
    )
    classes = {}
    for index, name in model.names.items():
        precision, recall, map50, map5095 = metrics.seg.class_result(int(index))
        classes[str(name)] = {
            "precision": float(precision), "recall": float(recall),
            "mask_map50": float(map50), "mask_map50_95": float(map5095),
        }
    report = {
        "schema": "rack_yolo26_validation/v1",
        "weights": str(args.weights.resolve()),
        "dataset": str(args.data.resolve()),
        "split": "val",
        "image_size": args.imgsz,
        "metrics": {key: float(value) for key, value in metrics.results_dict.items()},
        "classes": classes,
        "speed_ms": {key: float(value) for key, value in metrics.speed.items()},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
