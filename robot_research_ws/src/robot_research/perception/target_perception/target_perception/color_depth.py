"""Pure OpenCV/Numpy implementation of the first RGB-D target baseline."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class TargetEstimate:
    """One color target observation in the input optical frame."""

    position: tuple[float, float, float]
    size: tuple[float, float, float]
    position_variance: tuple[float, float, float]
    bbox_xywh: tuple[int, int, int, int]
    centroid_uv: tuple[float, float]
    contour_area: float
    valid_depth_points: int
    valid_depth_ratio: float
    confidence: float
    mask: np.ndarray


def depth_to_meters(depth: np.ndarray, encoding: str, depth_scale: float) -> np.ndarray:
    """Normalize common ROS depth encodings into float32 meters."""
    if depth.ndim != 2:
        raise ValueError("depth image must have one channel")
    normalized_encoding = encoding.upper()
    if normalized_encoding in {"16UC1", "MONO16"}:
        return depth.astype(np.float32) * float(depth_scale)
    if normalized_encoding == "32FC1":
        return depth.astype(np.float32, copy=False)
    raise ValueError(f"unsupported depth encoding: {encoding}")


def color_mask(
    bgr: np.ndarray,
    hsv_lower: tuple[int, int, int],
    hsv_upper: tuple[int, int, int],
    morphology_kernel: int,
) -> np.ndarray:
    """Return an HSV mask, including hue ranges that wrap through red."""
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("color image must be BGR with three channels")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low = np.asarray(hsv_lower, dtype=np.uint8)
    high = np.asarray(hsv_upper, dtype=np.uint8)
    if hsv_lower[0] <= hsv_upper[0]:
        mask = cv2.inRange(hsv, low, high)
    else:
        lower_wrap = np.asarray((0, hsv_lower[1], hsv_lower[2]), dtype=np.uint8)
        upper_wrap = np.asarray((179, hsv_upper[1], hsv_upper[2]), dtype=np.uint8)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_wrap, high),
            cv2.inRange(hsv, low, upper_wrap),
        )
    kernel_size = max(1, int(morphology_kernel))
    if kernel_size % 2 == 0:
        kernel_size += 1
    if kernel_size > 1:
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def estimate_color_target(
    bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    hsv_lower: tuple[int, int, int] = (35, 80, 50),
    hsv_upper: tuple[int, int, int] = (90, 255, 255),
    min_contour_area: float = 200.0,
    min_depth_points: int = 40,
    min_depth_m: float = 0.20,
    max_depth_m: float = 3.0,
    morphology_kernel: int = 5,
) -> TargetEstimate | None:
    """Detect the largest configured color region and robustly deproject it.

    HSV thresholding is only a replaceable 2D frontend.  The RGB-D geometry is
    shared with segmentation-model masks through :func:`estimate_mask_target`.
    """
    if bgr.shape[:2] != depth_m.shape:
        raise ValueError("color and aligned depth dimensions must match")
    mask = color_mask(bgr, hsv_lower, hsv_upper, morphology_kernel)
    return estimate_mask_target(
        mask,
        depth_m,
        intrinsics,
        min_mask_area=min_contour_area,
        min_depth_points=min_depth_points,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )


def estimate_mask_target(
    mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    min_mask_area: float = 200.0,
    min_depth_points: int = 40,
    min_depth_m: float = 0.20,
    max_depth_m: float = 3.0,
    prior_confidence: float = 1.0,
) -> TargetEstimate | None:
    """Robustly deproject the largest component of an arbitrary binary mask.

    This is the stable geometry boundary for future learned detection or
    segmentation frontends.  The input mask may be boolean or integer; all
    non-zero pixels are treated as foreground.
    """
    if mask.ndim != 2:
        raise ValueError("target mask must have one channel")
    if mask.shape != depth_m.shape:
        raise ValueError("target mask and aligned depth dimensions must match")
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if not 0.0 <= prior_confidence <= 1.0:
        raise ValueError("prior_confidence must be in [0, 1]")

    mask = np.where(mask != 0, 255, 0).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    contour_area = float(cv2.contourArea(contour))
    if contour_area < min_mask_area:
        return None

    object_mask = np.zeros_like(mask)
    cv2.drawContours(object_mask, [contour], -1, 255, thickness=cv2.FILLED)
    object_pixels = int(cv2.countNonZero(object_mask))
    finite = np.isfinite(depth_m)
    valid = (
        (object_mask > 0)
        & finite
        & (depth_m >= float(min_depth_m))
        & (depth_m <= float(max_depth_m))
    )
    rows, cols = np.nonzero(valid)
    if rows.size < min_depth_points:
        return None

    depths = depth_m[rows, cols].astype(np.float64)
    median_depth = float(np.median(depths))
    mad = float(np.median(np.abs(depths - median_depth)))
    depth_gate = max(0.02, 3.0 * 1.4826 * mad)
    inliers = np.abs(depths - median_depth) <= depth_gate
    rows = rows[inliers]
    cols = cols[inliers]
    depths = depths[inliers]
    if depths.size < min_depth_points:
        return None

    xs = (cols.astype(np.float64) - intrinsics.cx) * depths / intrinsics.fx
    ys = (rows.astype(np.float64) - intrinsics.cy) * depths / intrinsics.fy
    position = (float(np.median(xs)), float(np.median(ys)), float(np.median(depths)))

    x, y, width, height = cv2.boundingRect(contour)
    size_x = float(width * position[2] / intrinsics.fx)
    size_y = float(height * position[2] / intrinsics.fy)
    robust_sigma_z = max(0.005, 1.4826 * float(np.median(np.abs(depths - position[2]))))
    size_z = max(0.01, 2.0 * robust_sigma_z)
    pixel_sigma_x = position[2] / intrinsics.fx
    pixel_sigma_y = position[2] / intrinsics.fy
    variance = (
        float(pixel_sigma_x**2 + (position[0] * robust_sigma_z / position[2]) ** 2),
        float(pixel_sigma_y**2 + (position[1] * robust_sigma_z / position[2]) ** 2),
        float(robust_sigma_z**2),
    )

    moments = cv2.moments(contour)
    centroid = (
        float(moments["m10"] / moments["m00"]),
        float(moments["m01"] / moments["m00"]),
    )
    valid_ratio = float(depths.size / max(1, object_pixels))
    area_quality = min(1.0, contour_area / max(1.0, 4.0 * min_mask_area))
    depth_quality = min(1.0, valid_ratio / 0.70)
    confidence = float(prior_confidence * np.sqrt(area_quality * depth_quality))

    return TargetEstimate(
        position=position,
        size=(size_x, size_y, size_z),
        position_variance=variance,
        bbox_xywh=(int(x), int(y), int(width), int(height)),
        centroid_uv=centroid,
        contour_area=contour_area,
        valid_depth_points=int(depths.size),
        valid_depth_ratio=valid_ratio,
        confidence=confidence,
        mask=object_mask,
    )
