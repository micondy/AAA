"""Explainable camera-view visibility, occlusion and information-gain model."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import math

import numpy as np


@dataclass(frozen=True)
class PinholeCamera:
    """Pinhole intrinsics using the ROS optical-frame convention."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")


@dataclass(frozen=True)
class VisibilityConfig:
    """Thresholds and an initial D435-class observation-noise model."""

    near_plane_m: float = 0.10
    min_distance_m: float = 0.25
    max_distance_m: float = 4.0
    min_projected_area_ratio: float = 1.0e-4
    min_visible_fraction: float = 0.10
    min_incidence_cosine: float = 0.15
    max_occlusion_fraction: float = 0.80
    occlusion_tolerance_m: float = 0.03
    min_valid_depth_pixels: int = 20
    base_lateral_std_m: float = 0.003
    lateral_std_per_meter: float = 0.002
    base_depth_std_m: float = 0.005
    depth_std_per_meter_squared: float = 0.005
    quality_floor: float = 0.05

    def validate(self) -> None:
        positive = (
            self.near_plane_m,
            self.min_distance_m,
            self.max_distance_m,
            self.occlusion_tolerance_m,
            self.base_lateral_std_m,
            self.base_depth_std_m,
            self.quality_floor,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("positive visibility parameters must be finite")
        if self.max_distance_m <= self.min_distance_m:
            raise ValueError("max_distance_m must exceed min_distance_m")
        fractions = (
            self.min_projected_area_ratio,
            self.min_visible_fraction,
            self.min_incidence_cosine,
            self.max_occlusion_fraction,
            self.quality_floor,
        )
        if not all(0.0 <= value <= 1.0 for value in fractions):
            raise ValueError("visibility fractions must be in [0, 1]")
        if self.min_valid_depth_pixels < 1:
            raise ValueError("min_valid_depth_pixels must be positive")
        noise_terms = (
            self.lateral_std_per_meter,
            self.depth_std_per_meter_squared,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in noise_terms):
            raise ValueError("noise growth terms must be finite and non-negative")


@dataclass(frozen=True)
class ProjectedBox:
    """Continuous full and image-clipped target projection."""

    min_u: float
    min_v: float
    max_u: float
    max_v: float
    clipped_min_u: float
    clipped_min_v: float
    clipped_max_u: float
    clipped_max_v: float
    full_area_pixels: float
    visible_area_pixels: float
    frustum_fraction: float
    front_depth_m: float


@dataclass(frozen=True)
class DepthOcclusion:
    """Depth-based foreground occlusion estimate inside a target projection."""

    fraction: float
    known: bool
    valid_fraction: float
    valid_pixels: int
    occluding_pixels: int


@dataclass(frozen=True)
class VisibilityResult:
    """All interpretable terms used later by a next-best-view scorer."""

    visible: bool
    reasons: tuple[str, ...]
    distance_m: float
    projected_area_pixels: float
    projected_area_ratio: float
    frustum_fraction: float
    incidence_cosine: float
    incidence_known: bool
    occlusion_fraction: float
    occlusion_known: bool
    depth_valid_fraction: float
    visible_fraction: float
    expected_covariance: np.ndarray
    information_gain_nats: float | None
    projection: ProjectedBox | None


def _finite_vector(name: str, values, *, positive: bool = False) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite 3-vector")
    if positive and np.any(vector <= 0.0):
        raise ValueError(f"{name} values must be positive")
    return vector


def project_axis_aligned_box(
    camera: PinholeCamera,
    center_camera,
    size_xyz,
    near_plane_m: float = 0.10,
) -> ProjectedBox | None:
    """Project a camera-frame axis-aligned 3-D box into the image."""
    camera.validate()
    center = _finite_vector("center_camera", center_camera)
    size = _finite_vector("size_xyz", size_xyz, positive=True)
    if not math.isfinite(near_plane_m) or near_plane_m <= 0.0:
        raise ValueError("near_plane_m must be finite and positive")
    half = 0.5 * size
    front_depth = center[2] - half[2]
    if front_depth <= near_plane_m:
        return None

    image_points = []
    for signs in product((-1.0, 1.0), repeat=3):
        corner = center + half * np.asarray(signs)
        image_points.append(
            (
                camera.fx * corner[0] / corner[2] + camera.cx,
                camera.fy * corner[1] / corner[2] + camera.cy,
            )
        )
    pixels = np.asarray(image_points)
    min_u, min_v = np.min(pixels, axis=0)
    max_u, max_v = np.max(pixels, axis=0)
    full_area = max(0.0, max_u - min_u) * max(0.0, max_v - min_v)
    clipped_min_u = float(np.clip(min_u, 0.0, float(camera.width)))
    clipped_min_v = float(np.clip(min_v, 0.0, float(camera.height)))
    clipped_max_u = float(np.clip(max_u, 0.0, float(camera.width)))
    clipped_max_v = float(np.clip(max_v, 0.0, float(camera.height)))
    visible_area = max(0.0, clipped_max_u - clipped_min_u) * max(
        0.0, clipped_max_v - clipped_min_v
    )
    fraction = visible_area / full_area if full_area > 0.0 else 0.0
    return ProjectedBox(
        float(min_u),
        float(min_v),
        float(max_u),
        float(max_v),
        clipped_min_u,
        clipped_min_v,
        clipped_max_u,
        clipped_max_v,
        float(full_area),
        float(visible_area),
        float(np.clip(fraction, 0.0, 1.0)),
        float(front_depth),
    )


def estimate_depth_occlusion(
    depth_m: np.ndarray,
    projection: ProjectedBox,
    tolerance_m: float = 0.03,
    min_valid_pixels: int = 20,
) -> DepthOcclusion:
    """Estimate foreground occlusion from an aligned metric depth image."""
    depth = np.asarray(depth_m, dtype=float)
    if depth.ndim != 2:
        raise ValueError("depth_m must be a two-dimensional array")
    if not math.isfinite(tolerance_m) or tolerance_m < 0.0:
        raise ValueError("tolerance_m must be finite and non-negative")
    if min_valid_pixels < 1:
        raise ValueError("min_valid_pixels must be positive")
    height, width = depth.shape
    u0 = int(np.clip(math.floor(projection.clipped_min_u), 0, width))
    v0 = int(np.clip(math.floor(projection.clipped_min_v), 0, height))
    u1 = int(np.clip(math.ceil(projection.clipped_max_u), 0, width))
    v1 = int(np.clip(math.ceil(projection.clipped_max_v), 0, height))
    if u1 <= u0 or v1 <= v0:
        return DepthOcclusion(0.0, False, 0.0, 0, 0)
    region = depth[v0:v1, u0:u1]
    valid = np.isfinite(region) & (region > 0.0)
    valid_count = int(np.count_nonzero(valid))
    pixel_count = int(region.size)
    valid_fraction = valid_count / pixel_count if pixel_count else 0.0
    if valid_count < min_valid_pixels:
        return DepthOcclusion(0.0, False, valid_fraction, valid_count, 0)
    occluding = valid & (
        region < projection.front_depth_m - tolerance_m
    )
    occluding_count = int(np.count_nonzero(occluding))
    return DepthOcclusion(
        occluding_count / valid_count,
        True,
        valid_fraction,
        valid_count,
        occluding_count,
    )


def gaussian_information_gain(prior_covariance, measurement_covariance) -> float:
    """Return the expected static-Gaussian covariance reduction in nats."""
    prior = np.asarray(prior_covariance, dtype=float)
    measurement = np.asarray(measurement_covariance, dtype=float)
    if prior.shape != (3, 3) or measurement.shape != (3, 3):
        raise ValueError("covariances must be 3x3 matrices")
    if not np.all(np.isfinite(prior)) or not np.all(np.isfinite(measurement)):
        raise ValueError("covariances must be finite")

    def regularize(matrix):
        symmetric = 0.5 * (matrix + matrix.T)
        values, vectors = np.linalg.eigh(symmetric)
        if np.any(values <= 0.0):
            raise ValueError("covariances must be positive definite")
        return vectors @ np.diag(values) @ vectors.T

    prior = regularize(prior)
    measurement = regularize(measurement)
    posterior = np.linalg.inv(np.linalg.inv(prior) + np.linalg.inv(measurement))
    prior_sign, prior_logdet = np.linalg.slogdet(prior)
    posterior_sign, posterior_logdet = np.linalg.slogdet(posterior)
    if prior_sign <= 0.0 or posterior_sign <= 0.0:
        raise ValueError("covariances must have positive determinants")
    return max(0.0, 0.5 * float(prior_logdet - posterior_logdet))


def evaluate_view(
    camera: PinholeCamera,
    center_camera,
    size_xyz,
    *,
    config: VisibilityConfig | None = None,
    depth_m: np.ndarray | None = None,
    external_occlusion_fraction: float | None = None,
    surface_normal_camera=None,
    prior_covariance=None,
) -> VisibilityResult:
    """Evaluate one target from one actual or hypothetical camera pose."""
    config = config or VisibilityConfig()
    config.validate()
    center = _finite_vector("center_camera", center_camera)
    size = _finite_vector("size_xyz", size_xyz, positive=True)
    if external_occlusion_fraction is not None and (
        not math.isfinite(external_occlusion_fraction)
        or not 0.0 <= external_occlusion_fraction <= 1.0
    ):
        raise ValueError("external_occlusion_fraction must be in [0, 1]")
    distance = float(np.linalg.norm(center))
    projection = project_axis_aligned_box(
        camera, center, size, config.near_plane_m
    )

    incidence_known = surface_normal_camera is not None
    incidence = 1.0
    if incidence_known:
        normal = _finite_vector("surface_normal_camera", surface_normal_camera)
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm <= 0.0 or distance <= 0.0:
            raise ValueError("surface normal and viewing direction must be non-zero")
        incidence = max(0.0, float(np.dot(normal / normal_norm, -center / distance)))

    if projection is None:
        projected_area = 0.0
        projected_ratio = 0.0
        frustum_fraction = 0.0
        occlusion = DepthOcclusion(0.0, False, 0.0, 0, 0)
    else:
        projected_area = projection.visible_area_pixels
        projected_ratio = projected_area / (camera.width * camera.height)
        frustum_fraction = projection.frustum_fraction
        depth_occlusion = (
            estimate_depth_occlusion(
                depth_m,
                projection,
                config.occlusion_tolerance_m,
                config.min_valid_depth_pixels,
            )
            if depth_m is not None
            else DepthOcclusion(0.0, False, 0.0, 0, 0)
        )
        if external_occlusion_fraction is None:
            occlusion = depth_occlusion
        else:
            combined = 1.0 - (1.0 - depth_occlusion.fraction) * (
                1.0 - external_occlusion_fraction
            )
            occlusion = DepthOcclusion(
                combined,
                True,
                depth_occlusion.valid_fraction,
                depth_occlusion.valid_pixels,
                depth_occlusion.occluding_pixels,
            )

    visible_fraction = frustum_fraction * (1.0 - occlusion.fraction)
    if incidence_known:
        visible_fraction *= incidence
    quality = max(config.quality_floor, visible_fraction)
    lateral_std = (
        config.base_lateral_std_m + config.lateral_std_per_meter * distance
    ) / math.sqrt(quality)
    depth_std = (
        config.base_depth_std_m
        + config.depth_std_per_meter_squared * center[2] * center[2]
    ) / math.sqrt(quality)
    expected_covariance = np.diag(
        (lateral_std * lateral_std, lateral_std * lateral_std, depth_std * depth_std)
    )

    reasons = []
    if projection is None or frustum_fraction <= 0.0:
        reasons.append("outside_frustum_or_near_plane")
    if distance < config.min_distance_m:
        reasons.append("too_close")
    if distance > config.max_distance_m:
        reasons.append("too_far")
    if projected_ratio < config.min_projected_area_ratio:
        reasons.append("projection_too_small")
    if incidence_known and incidence < config.min_incidence_cosine:
        reasons.append("unfavorable_incidence")
    if occlusion.known and occlusion.fraction > config.max_occlusion_fraction:
        reasons.append("occluded")
    if visible_fraction < config.min_visible_fraction:
        reasons.append("insufficient_visible_fraction")
    visible = not reasons
    information_gain = None
    if prior_covariance is not None:
        information_gain = (
            gaussian_information_gain(prior_covariance, expected_covariance)
            if visible
            else 0.0
        )
    return VisibilityResult(
        visible,
        tuple(reasons),
        distance,
        projected_area,
        projected_ratio,
        frustum_fraction,
        incidence,
        incidence_known,
        occlusion.fraction,
        occlusion.known,
        occlusion.valid_fraction,
        float(np.clip(visible_fraction, 0.0, 1.0)),
        expected_covariance,
        information_gain,
        projection,
    )
