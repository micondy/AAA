"""Convert frame-local YOLO instance predictions to the rack mono16 contract."""

from __future__ import annotations

import cv2
import numpy as np


def validate_rack_classes(names: dict[int, str] | list[str]) -> None:
    """Reject generic COCO weights before they can masquerade as rack detections."""
    mapping = dict(enumerate(names)) if isinstance(names, list) else names
    if {int(key): str(value).lower() for key, value in mapping.items()} != {0: "bed", 1: "post"}:
        raise ValueError("rack weights must have exactly class 0=bed and class 1=post")


def _array(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    return np.asarray(value)


def encode_yolo_result(
    result, image_shape: tuple[int, int], min_pixels: int = 80
) -> tuple[np.ndarray, list[dict]]:
    """Assign ephemeral IDs; high-confidence masks win overlap pixels."""
    output = np.zeros(image_shape, dtype=np.uint16)
    if result.boxes is None or result.masks is None:
        return output, []
    classes = _array(result.boxes.cls).astype(int).reshape(-1)
    scores = _array(result.boxes.conf).astype(float).reshape(-1)
    masks = _array(result.masks.data)
    if masks.ndim != 3 or len(classes) != len(scores) or len(scores) != len(masks):
        raise ValueError("YOLO boxes and masks have inconsistent lengths")
    instances: list[dict] = []
    next_id = {0: 1, 1: 10000}
    limit = {0: 9999, 1: 65535}
    for index in np.argsort(-scores):
        class_id = int(classes[index])
        if class_id not in next_id or next_id[class_id] > limit[class_id]:
            continue
        binary = masks[index] > 0.5
        if binary.shape != image_shape:
            binary = cv2.resize(
                binary.astype(np.uint8), (image_shape[1], image_shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        unclaimed = binary & (output == 0)
        pixels = int(np.count_nonzero(unclaimed))
        if pixels < min_pixels:
            continue
        mask_id = next_id[class_id]
        next_id[class_id] += 1
        output[unclaimed] = mask_id
        instances.append({
            "mask_id": mask_id,
            "label": "bed" if class_id == 0 else "post",
            "confidence": round(float(scores[index]), 4),
            "pixels": pixels,
        })
    return output, instances


def colorize_mask(mask: np.ndarray) -> np.ndarray:
    color = np.zeros((*mask.shape, 3), dtype=np.uint8)
    for mask_id in np.unique(mask):
        if mask_id == 0:
            continue
        if mask_id >= 10000:
            bgr = (220, 220, 220)
        else:
            bgr = (
                40 + (int(mask_id) * 67) % 180,
                50 + (int(mask_id) * 97) % 170,
                40 + (int(mask_id) * 137) % 180,
            )
        color[mask == mask_id] = bgr
    return color
