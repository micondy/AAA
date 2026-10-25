from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "tools"))
from train_rack_yolo26 import check_dataset  # noqa: E402


def test_dataset_guard_rejects_missing_real_labels(tmp_path):
    config = tmp_path / "data.yaml"
    config.write_text(
        f"path: {tmp_path}\ntrain: images/train\nval: images/val\nnames: {{0: bed, 1: post}}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="missing train images or labels"):
        check_dataset(config)


def test_dataset_guard_accepts_two_labeled_splits(tmp_path):
    config = tmp_path / "data.yaml"
    config.write_text(
        f"path: {tmp_path}\ntrain: images/train\nval: images/val\nnames: {{0: bed, 1: post}}\n",
        encoding="utf-8",
    )
    for split in ("train", "val"):
        image_dir = tmp_path / "images" / split
        label_dir = tmp_path / "labels" / split
        image_dir.mkdir(parents=True)
        label_dir.mkdir(parents=True)
        (image_dir / "frame_001.png").touch()
        (label_dir / "frame_001.txt").write_text("0 0.1 0.1 0.9 0.1 0.9 0.9\n", encoding="utf-8")
    assert check_dataset(config)["names"] == {0: "bed", 1: "post"}
