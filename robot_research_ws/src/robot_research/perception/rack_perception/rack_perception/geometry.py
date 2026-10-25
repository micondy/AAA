"""Pure NumPy depth deprojection and rack-layer geometry fitting."""

from __future__ import annotations

import math

import numpy as np

from rack_perception.model import LayerObservation


def depth_to_meters(depth: np.ndarray, encoding: str, scale_16u: float) -> np.ndarray:
    if depth.ndim != 2:
        raise ValueError("depth image must have one channel")
    normalized = encoding.upper()
    if normalized in {"16UC1", "MONO16"}:
        return depth.astype(np.float32) * float(scale_16u)
    if normalized == "32FC1":
        return depth.astype(np.float32, copy=False)
    raise ValueError(f"unsupported depth encoding: {encoding}")


def deproject_mask(
    mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    min_depth_m: float,
    max_depth_m: float,
    max_points: int = 8000,
) -> np.ndarray:
    """Return optical-frame XYZ for finite masked depth pixels."""
    if mask.shape != depth_m.shape:
        raise ValueError("mask and depth dimensions must match")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    valid = (
        np.asarray(mask, dtype=bool)
        & np.isfinite(depth_m)
        & (depth_m >= float(min_depth_m))
        & (depth_m <= float(max_depth_m))
    )
    rows, cols = np.nonzero(valid)
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if rows.size > max_points:
        selected = np.linspace(0, rows.size - 1, max_points, dtype=np.int64)
        rows, cols = rows[selected], cols[selected]
    z = depth_m[rows, cols].astype(np.float64)
    x = (cols.astype(np.float64) - cx) * z / fx
    y = (rows.astype(np.float64) - cy) * z / fy
    return np.column_stack((x, y, z))


def transform_points(
    points: np.ndarray, rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be Nx3")
    return points @ np.asarray(rotation, dtype=np.float64).T + np.asarray(
        translation, dtype=np.float64
    )


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def fit_layer(
    points_map: np.ndarray,
    camera_map: tuple[float, float, float],
    source_id: int,
    min_points: int = 80,
    top_band_m: float = 0.04,
    bed_length_m: float = 2.0,
    axis_prior_yaw_rad: float | None = None,
) -> LayerObservation | None:
    """Fit the top boundary and camera-facing edge of one segmented bed.

    Semantic pixels can include the tray fascia.  The high-Z band retains the
    planting surface and its front edge while rejecting most vertical pixels.
    """
    points = np.asarray(points_map, dtype=np.float64)
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if len(points) < min_points:
        return None

    center0 = np.median(points, axis=0)
    radial = np.linalg.norm(points - center0, axis=1)
    mad = np.median(np.abs(radial - np.median(radial)))
    if mad > 1.0e-6:
        points = points[radial <= np.median(radial) + 4.5 * 1.4826 * mad]
    if len(points) < min_points:
        return None

    top_z = float(np.quantile(points[:, 2], 0.96))
    top = points[points[:, 2] >= top_z - float(top_band_m)]
    if len(top) < min_points:
        top = points[points[:, 2] >= np.quantile(points[:, 2], 0.82)]
    if len(top) < min_points:
        return None

    xy = top[:, :2]
    center_xy = np.median(xy, axis=0)
    if axis_prior_yaw_rad is None:
        covariance = np.cov(xy - center_xy, rowvar=False)
        values, vectors = np.linalg.eigh(covariance)
        axis = vectors[:, int(np.argmax(values))]
    else:
        if not math.isfinite(axis_prior_yaw_rad):
            raise ValueError("axis_prior_yaw_rad must be finite")
        axis = np.asarray((math.cos(axis_prior_yaw_rad), math.sin(axis_prior_yaw_rad)))
    dominant = int(np.argmax(np.abs(axis)))
    if axis[dominant] < 0.0:
        axis = -axis
    cross = np.asarray((-axis[1], axis[0]))
    along = (xy - center_xy) @ axis
    across = (xy - center_xy) @ cross
    lo_a, hi_a = np.quantile(along, (0.02, 0.98))
    lo_c, hi_c = np.quantile(across, (0.02, 0.98))
    if hi_a - lo_a < 0.15 or hi_c - lo_c < 0.05:
        return None

    camera_xy = np.asarray(camera_map[:2], dtype=np.float64)
    camera_cross = float((camera_xy - center_xy) @ cross)
    front_coordinate = hi_c if abs(camera_cross - hi_c) < abs(camera_cross - lo_c) else lo_c
    normal = cross if front_coordinate == hi_c else -cross
    front_center_xy = center_xy + axis * ((lo_a + hi_a) / 2.0) + cross * front_coordinate
    fitted_center_xy = center_xy + axis * ((lo_a + hi_a) / 2.0) + cross * ((lo_c + hi_c) / 2.0)
    corners = tuple(
        (
            float((center_xy + axis * a + cross * c)[0]),
            float((center_xy + axis * a + cross * c)[1]),
            top_z,
        )
        for a, c in ((lo_a, lo_c), (hi_a, lo_c), (hi_a, hi_c), (lo_a, hi_c))
    )
    spread_z = float(np.median(np.abs(top[:, 2] - np.median(top[:, 2]))))
    planarity = math.exp(-spread_z / 0.02)
    confidence = float(min(1.0, len(top) / 1200.0) * planarity)
    observed_ratio = float(min(1.0, (hi_a - lo_a) / bed_length_m))
    return LayerObservation(
        source_id=int(source_id),
        center=(float(fitted_center_xy[0]), float(fitted_center_xy[1]), top_z),
        axis=(float(axis[0]), float(axis[1])),
        front_center=(float(front_center_xy[0]), float(front_center_xy[1]), top_z),
        front_normal=(float(normal[0]), float(normal[1])),
        boundary=corners,
        confidence=confidence,
        observed_ratio=observed_ratio,
        point_count=int(len(top)),
    )
