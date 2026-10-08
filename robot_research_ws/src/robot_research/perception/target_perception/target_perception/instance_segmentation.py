"""Model-agnostic instance predictions and torchvision output parsing."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Mapping, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class InstancePrediction:
    """One filtered 2D instance prediction in image coordinates."""

    class_index: int
    class_id: str
    score: float
    bbox_xyxy: tuple[float, float, float, float]
    mask: np.ndarray


def draw_instance_overlay(
    bgr: np.ndarray,
    predictions: Sequence[InstancePrediction],
    alpha: float = 0.45,
) -> np.ndarray:
    """Draw deterministic masks, boxes, class names and scores on a BGR image."""
    if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
        raise ValueError("overlay input must be an H x W x 3 uint8 BGR image")
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be in [0, 1]")

    overlay = bgr.copy()
    height, width = bgr.shape[:2]
    palette = (
        (66, 135, 245),
        (80, 200, 120),
        (230, 150, 60),
        (180, 90, 220),
        (60, 210, 220),
        (220, 90, 100),
    )
    for rank, prediction in enumerate(predictions):
        if prediction.mask.shape != (height, width):
            raise ValueError("instance mask dimensions must match the image")
        color = palette[rank % len(palette)]
        mask = prediction.mask.astype(bool)
        if np.any(mask):
            colored = np.asarray(color, dtype=np.float32)
            overlay[mask] = np.clip(
                (1.0 - alpha) * overlay[mask].astype(np.float32) + alpha * colored,
                0,
                255,
            ).astype(np.uint8)
        x1, y1, x2, y2 = (
            int(round(value)) for value in prediction.bbox_xyxy
        )
        x1, x2 = np.clip((x1, x2), 0, width - 1)
        y1, y2 = np.clip((y1, y2), 0, height - 1)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
        label = f"{prediction.class_id} {prediction.score:.2f}"
        cv2.putText(
            overlay,
            label,
            (x1, max(15, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
    return overlay


def _as_numpy(value: object) -> np.ndarray:
    """Convert numpy arrays or CPU/GPU tensor-like values without importing torch."""
    candidate = value
    for method in ("detach", "cpu"):
        operation = getattr(candidate, method, None)
        if callable(operation):
            candidate = operation()
    return np.asarray(candidate)


def parse_torchvision_instances(
    output: Mapping[str, object],
    class_names: Sequence[str],
    score_threshold: float = 0.5,
    mask_threshold: float = 0.5,
    allowed_class_indices: set[int] | None = None,
) -> list[InstancePrediction]:
    """Validate and filter one torchvision instance-segmentation result.

    Expected keys follow torchvision detection models: ``boxes`` (N×4),
    ``labels`` (N), ``scores`` (N) and ``masks`` (N×1×H×W or N×H×W).
    """
    if not 0.0 <= score_threshold <= 1.0:
        raise ValueError("score_threshold must be in [0, 1]")
    if not 0.0 <= mask_threshold <= 1.0:
        raise ValueError("mask_threshold must be in [0, 1]")
    missing = {"boxes", "labels", "scores", "masks"} - output.keys()
    if missing:
        raise ValueError(f"model output is missing keys: {sorted(missing)}")

    boxes = _as_numpy(output["boxes"])
    labels = _as_numpy(output["labels"]).reshape(-1)
    scores = _as_numpy(output["scores"]).reshape(-1)
    masks = _as_numpy(output["masks"])
    if boxes.ndim != 2 or boxes.shape[1] != 4:
        raise ValueError("boxes must have shape N x 4")
    if masks.ndim == 4 and masks.shape[1] == 1:
        masks = masks[:, 0]
    if masks.ndim != 3:
        raise ValueError("masks must have shape N x 1 x H x W or N x H x W")
    count = boxes.shape[0]
    if not (labels.size == scores.size == masks.shape[0] == count):
        raise ValueError("model output arrays have inconsistent instance counts")

    predictions: list[InstancePrediction] = []
    for index in range(count):
        class_index = int(labels[index])
        score = float(scores[index])
        if score < score_threshold:
            continue
        if allowed_class_indices is not None and class_index not in allowed_class_indices:
            continue
        if class_index < 0 or class_index >= len(class_names):
            raise ValueError(f"class index {class_index} has no configured name")
        box = tuple(float(value) for value in boxes[index])
        if not all(np.isfinite(box)) or box[2] <= box[0] or box[3] <= box[1]:
            raise ValueError(f"instance {index} has an invalid bounding box")
        predictions.append(
            InstancePrediction(
                class_index=class_index,
                class_id=str(class_names[class_index]),
                score=score,
                bbox_xyxy=box,
                mask=np.asarray(masks[index] >= mask_threshold, dtype=bool),
            )
        )
    return predictions


class TorchvisionMaskRCNNBackend:
    """Lazy torchvision Mask R-CNN inference with explicit local weights."""

    def __init__(
        self,
        weights_path: str | Path,
        device: str = "auto",
        score_threshold: float = 0.5,
        mask_threshold: float = 0.5,
        allowed_class_names: Sequence[str] = (),
    ) -> None:
        path = Path(weights_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(
                f"instance-segmentation weights do not exist: {path}"
            )
        import torch
        from torchvision.models.detection import (
            MaskRCNN_ResNet50_FPN_V2_Weights,
            maskrcnn_resnet50_fpn_v2,
        )

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(f"CUDA device requested but unavailable: {device}")
        self.device = torch.device(device)
        self.class_names = tuple(
            MaskRCNN_ResNet50_FPN_V2_Weights.DEFAULT.meta["categories"]
        )
        requested_classes = tuple(allowed_class_names)
        if "*" in requested_classes:
            if requested_classes != ("*",):
                raise ValueError("'*' cannot be combined with named COCO classes")
            self.allowed_class_indices = None
        else:
            unknown = sorted(set(requested_classes) - set(self.class_names))
            if unknown:
                raise ValueError(f"unknown COCO classes: {unknown}")
            self.allowed_class_indices = (
                {self.class_names.index(name) for name in requested_classes}
                if requested_classes
                else None
            )
        self.score_threshold = float(score_threshold)
        self.mask_threshold = float(mask_threshold)
        self.weights_path = path
        self.weights_sha256 = self._sha256(path)

        model = maskrcnn_resnet50_fpn_v2(weights=None, num_classes=91)
        state = torch.load(path, map_location="cpu", weights_only=True)
        if isinstance(state, Mapping) and "state_dict" in state:
            state = state["state_dict"]
        model.load_state_dict(state)
        self.model = model.eval().to(self.device)
        self._torch = torch

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def infer(self, bgr: np.ndarray) -> list[InstancePrediction]:
        """Run one BGR uint8 image and return filtered CPU predictions."""
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("model input must be an H x W x 3 uint8 BGR image")
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        tensor = (
            self._torch.from_numpy(rgb)
            .permute(2, 0, 1)
            .to(self.device, dtype=self._torch.float32)
            / 255.0
        )
        with self._torch.inference_mode():
            output = self.model([tensor])[0]
        return parse_torchvision_instances(
            output,
            self.class_names,
            score_threshold=self.score_threshold,
            mask_threshold=self.mask_threshold,
            allowed_class_indices=self.allowed_class_indices,
        )
