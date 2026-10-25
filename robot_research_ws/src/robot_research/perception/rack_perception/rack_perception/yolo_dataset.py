"""Convert Isaac-derived rack instance masks into YOLO polygon annotations."""

from __future__ import annotations

import cv2
import numpy as np


def _polygon(contour: np.ndarray, width: int, height: int) -> list[float] | None:
    if cv2.contourArea(contour) <= 0:
        return None
    perimeter = cv2.arcLength(contour, True)
    points = cv2.approxPolyDP(contour, max(1.0, 0.002 * perimeter), True).reshape(-1, 2)
    if len(points) < 3:
        return None
    normalized = np.column_stack((points[:, 0] / width, points[:, 1] / height))
    return np.clip(normalized, 0.0, 1.0).reshape(-1).tolist()


def mask_to_yolo_rows(
    mask: np.ndarray, min_bed_pixels: int = 80, min_post_pixels: int = 20
) -> tuple[list[str], dict[str, int]]:
    """One bed row per layer; split disconnected post pixels into visible posts."""
    if mask.ndim != 2 or mask.dtype != np.uint16:
        raise ValueError("expected a 2D mono16 rack instance mask")
    height, width = mask.shape
    rows: list[str] = []
    counts = {"bed": 0, "post": 0}
    for mask_id in np.unique(mask):
        if mask_id == 0:
            continue
        category = "bed" if mask_id < 10000 else "post"
        binary = (mask == mask_id).astype(np.uint8)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if category == "bed":
            # A partially occluded bed can have several islands. YOLO polygon
            # rows cannot express a disconnected single instance, so keep its
            # largest visible component and record one class-0 bed instance.
            contours = contours[:1]
        for contour in contours:
            minimum = min_bed_pixels if category == "bed" else min_post_pixels
            if cv2.contourArea(contour) < minimum:
                continue
            coordinates = _polygon(contour, width, height)
            if coordinates is None:
                continue
            class_id = 0 if category == "bed" else 1
            rows.append(" ".join([str(class_id)] + [f"{value:.6f}" for value in coordinates]))
            counts[category] += 1
    return rows, counts
