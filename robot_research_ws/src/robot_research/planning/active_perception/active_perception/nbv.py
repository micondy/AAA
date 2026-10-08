"""Finite mobile-base next-best-view candidate generation and scoring."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .visibility import evaluate_view, PinholeCamera, VisibilityConfig, VisibilityResult


def wrap_angle(angle: float) -> float:
    """Wrap one angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class BaseCandidate:
    """One planar base pose that points toward the target."""

    index: int
    ring_index: int
    x: float
    y: float
    yaw: float
    radius_m: float
    azimuth_rad: float


@dataclass(frozen=True)
class NBVConfig:
    """Candidate layout, collision thresholds and explainable score weights."""

    radii_m: tuple[float, ...] = (0.6, 0.8, 1.0)
    samples_per_ring: int = 12
    robot_radius_m: float = 0.32
    safety_margin_m: float = 0.08
    occupied_threshold: int = 50
    unknown_is_occupied: bool = True
    target_ray_exclusion_m: float = 0.15
    desired_clearance_m: float = 0.60
    max_path_length_m: float = 5.0
    information_gain_weight: float = 1.0
    visible_fraction_weight: float = 0.5
    clearance_weight: float = 0.2
    path_length_weight: float = 0.15
    turn_weight: float = 0.10
    occlusion_weight: float = 0.5

    def validate(self) -> None:
        if not self.radii_m or not all(
            math.isfinite(radius) and radius > 0.0 for radius in self.radii_m
        ):
            raise ValueError("radii_m must contain finite positive values")
        if self.samples_per_ring < 3:
            raise ValueError("samples_per_ring must be at least 3")
        positive = (
            self.robot_radius_m,
            self.desired_clearance_m,
            self.max_path_length_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("positive NBV parameters must be finite")
        nonnegative = (
            self.safety_margin_m,
            self.target_ray_exclusion_m,
            self.information_gain_weight,
            self.visible_fraction_weight,
            self.clearance_weight,
            self.path_length_weight,
            self.turn_weight,
            self.occlusion_weight,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in nonnegative):
            raise ValueError("NBV margins and weights must be finite and non-negative")
        if not 0 <= self.occupied_threshold <= 100:
            raise ValueError("occupied_threshold must be in [0, 100]")


@dataclass(frozen=True)
class OccupancyGrid2D:
    """ROS-compatible row-major occupancy data with a planar map origin."""

    data: np.ndarray
    resolution_m: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def __post_init__(self) -> None:
        data = np.asarray(self.data, dtype=np.int16)
        if data.ndim != 2:
            raise ValueError("occupancy data must be a two-dimensional array")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be finite and positive")
        if not all(
            math.isfinite(value)
            for value in (self.origin_x, self.origin_y, self.origin_yaw)
        ):
            raise ValueError("map origin must be finite")
        object.__setattr__(self, "data", data)

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    def world_to_grid(self, x: float, y: float) -> tuple[float, float]:
        """Convert world coordinates to continuous grid coordinates."""
        dx = float(x) - self.origin_x
        dy = float(y) - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        return local_x / self.resolution_m, local_y / self.resolution_m

    def grid_to_world(self, column: float, row: float) -> tuple[float, float]:
        """Convert continuous grid coordinates to world coordinates."""
        local_x = float(column) * self.resolution_m
        local_y = float(row) * self.resolution_m
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )

    def _blocked_mask(self, threshold: int, unknown_is_occupied: bool) -> np.ndarray:
        occupied = self.data >= threshold
        return occupied | (self.data < 0) if unknown_is_occupied else occupied

    def footprint_is_free(
        self,
        x: float,
        y: float,
        radius_m: float,
        *,
        occupied_threshold: int = 50,
        unknown_is_occupied: bool = True,
    ) -> bool:
        """Return whether a circular footprint lies inside free grid cells."""
        if radius_m < 0.0 or not math.isfinite(radius_m):
            raise ValueError("radius_m must be finite and non-negative")
        column, row = self.world_to_grid(x, y)
        radius_cells = radius_m / self.resolution_m
        min_column = math.floor(column - radius_cells)
        max_column = math.ceil(column + radius_cells)
        min_row = math.floor(row - radius_cells)
        max_row = math.ceil(row + radius_cells)
        if (
            min_column < 0
            or min_row < 0
            or max_column >= self.width
            or max_row >= self.height
        ):
            return False
        blocked = self._blocked_mask(occupied_threshold, unknown_is_occupied)
        for grid_row in range(min_row, max_row + 1):
            for grid_column in range(min_column, max_column + 1):
                if math.hypot(
                    grid_column + 0.5 - column, grid_row + 0.5 - row
                ) <= radius_cells:
                    if blocked[grid_row, grid_column]:
                        return False
        return True

    def clearance_m(
        self,
        x: float,
        y: float,
        max_distance_m: float,
        *,
        occupied_threshold: int = 50,
        unknown_is_occupied: bool = True,
    ) -> float:
        """Return center-to-nearest-blocked-cell distance, capped at a limit."""
        column, row = self.world_to_grid(x, y)
        if not (0.0 <= column < self.width and 0.0 <= row < self.height):
            return 0.0
        blocked = self._blocked_mask(occupied_threshold, unknown_is_occupied)
        indices = np.argwhere(blocked)
        if indices.size == 0:
            return float(max_distance_m)
        distances_cells = np.hypot(
            indices[:, 1] + 0.5 - column, indices[:, 0] + 0.5 - row
        )
        return min(float(np.min(distances_cells)) * self.resolution_m, max_distance_m)

    def ray_occlusion_fraction(
        self,
        start_xy: tuple[float, float],
        end_xy: tuple[float, float],
        *,
        occupied_threshold: int = 50,
        unknown_is_occupied: bool = True,
        end_exclusion_m: float = 0.15,
    ) -> float:
        """Sample a 2-D line of sight and return its blocked sample fraction."""
        start = np.asarray(start_xy, dtype=float)
        end = np.asarray(end_xy, dtype=float)
        if start.shape != (2,) or end.shape != (2,) or not (
            np.all(np.isfinite(start)) and np.all(np.isfinite(end))
        ):
            raise ValueError("ray endpoints must be finite 2-vectors")
        delta = end - start
        distance = float(np.linalg.norm(delta))
        if distance <= end_exclusion_m:
            return 0.0
        usable_distance = distance - max(0.0, end_exclusion_m)
        samples = max(2, int(math.ceil(usable_distance / (0.5 * self.resolution_m))))
        blocked = self._blocked_mask(occupied_threshold, unknown_is_occupied)
        blocked_count = 0
        valid_count = 0
        for fraction in np.linspace(0.0, usable_distance / distance, samples):
            point = start + delta * fraction
            column, row = self.world_to_grid(point[0], point[1])
            grid_column = math.floor(column)
            grid_row = math.floor(row)
            valid_count += 1
            if not (0 <= grid_column < self.width and 0 <= grid_row < self.height):
                blocked_count += 1
            elif blocked[grid_row, grid_column]:
                blocked_count += 1
        return blocked_count / valid_count if valid_count else 0.0


@dataclass(frozen=True)
class CandidateScore:
    """One candidate with all score terms and rejection explanations."""

    candidate: BaseCandidate
    visibility: VisibilityResult
    footprint_free: bool
    reachable: bool
    clearance_m: float
    map_occlusion_fraction: float
    path_length_m: float
    yaw_change_rad: float
    information_gain_term: float
    visible_fraction_term: float
    clearance_term: float
    path_penalty: float
    turn_penalty: float
    occlusion_penalty: float
    total_score: float
    rejection_reasons: tuple[str, ...]
    path_source: str


def generate_base_candidates(
    target_xy,
    *,
    radii_m: tuple[float, ...] = (0.6, 0.8, 1.0),
    samples_per_ring: int = 12,
    azimuth_offset_rad: float = 0.0,
) -> list[BaseCandidate]:
    """Generate deterministic rings of base poses facing the target."""
    target = np.asarray(target_xy, dtype=float)
    if target.shape != (2,) or not np.all(np.isfinite(target)):
        raise ValueError("target_xy must be a finite 2-vector")
    config = NBVConfig(radii_m=radii_m, samples_per_ring=samples_per_ring)
    config.validate()
    candidates = []
    index = 0
    for ring_index, radius in enumerate(radii_m):
        for sample in range(samples_per_ring):
            azimuth = wrap_angle(
                azimuth_offset_rad + 2.0 * math.pi * sample / samples_per_ring
            )
            x = float(target[0] + radius * math.cos(azimuth))
            y = float(target[1] + radius * math.sin(azimuth))
            yaw = math.atan2(target[1] - y, target[0] - x)
            candidates.append(
                BaseCandidate(index, ring_index, x, y, yaw, radius, azimuth)
            )
            index += 1
    return candidates


def _validated_transform(name: str, transform) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1.0e-9):
        raise ValueError(f"{name} must be homogeneous")
    return matrix


def target_in_candidate_camera(
    candidate: BaseCandidate,
    target_world,
    size_world,
    camera_from_base,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform target center/covariance axes into a candidate camera frame."""
    target = np.asarray(target_world, dtype=float)
    size = np.asarray(size_world, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("target_world must be a finite 3-vector")
    if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0.0):
        raise ValueError("size_world must contain three finite positive values")
    camera_from_base = _validated_transform("camera_from_base", camera_from_base)
    cosine = math.cos(candidate.yaw)
    sine = math.sin(candidate.yaw)
    world_from_base = np.array(
        (
            (cosine, -sine, 0.0, candidate.x),
            (sine, cosine, 0.0, candidate.y),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
    )
    camera_from_world = camera_from_base @ np.linalg.inv(world_from_base)
    center_camera = (camera_from_world @ np.append(target, 1.0))[:3]
    size_camera = np.abs(camera_from_world[:3, :3]) @ size
    return center_camera, size_camera, camera_from_world


def evaluate_base_candidates(
    candidates: list[BaseCandidate],
    *,
    current_base_pose,
    target_world,
    target_size_world,
    prior_covariance,
    camera: PinholeCamera,
    camera_from_base,
    occupancy_grid: OccupancyGrid2D | None = None,
    config: NBVConfig | None = None,
    visibility_config: VisibilityConfig | None = None,
    path_lengths_m: dict[int, float | None] | None = None,
) -> list[CandidateScore]:
    """Filter and rank finite candidates using geometry, map and path costs."""
    config = config or NBVConfig()
    config.validate()
    current = np.asarray(current_base_pose, dtype=float)
    target = np.asarray(target_world, dtype=float)
    if current.shape != (3,) or not np.all(np.isfinite(current)):
        raise ValueError("current_base_pose must be finite (x, y, yaw)")
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("target_world must be a finite 3-vector")
    scores = []
    footprint_radius = config.robot_radius_m + config.safety_margin_m
    for candidate in candidates:
        footprint_free = True
        clearance = config.desired_clearance_m
        map_occlusion_fraction = 0.0
        if occupancy_grid is not None:
            footprint_free = occupancy_grid.footprint_is_free(
                candidate.x,
                candidate.y,
                footprint_radius,
                occupied_threshold=config.occupied_threshold,
                unknown_is_occupied=config.unknown_is_occupied,
            )
            clearance = occupancy_grid.clearance_m(
                candidate.x,
                candidate.y,
                config.desired_clearance_m,
                occupied_threshold=config.occupied_threshold,
                unknown_is_occupied=config.unknown_is_occupied,
            )
            map_occlusion_fraction = occupancy_grid.ray_occlusion_fraction(
                (candidate.x, candidate.y),
                (target[0], target[1]),
                occupied_threshold=config.occupied_threshold,
                unknown_is_occupied=config.unknown_is_occupied,
                end_exclusion_m=config.target_ray_exclusion_m,
            )
        center_camera, size_camera, _ = target_in_candidate_camera(
            candidate, target, target_size_world, camera_from_base
        )
        visibility = evaluate_view(
            camera,
            center_camera,
            size_camera,
            config=visibility_config,
            external_occlusion_fraction=(
                map_occlusion_fraction
                if occupancy_grid is not None
                else None
            ),
            prior_covariance=prior_covariance,
        )
        euclidean_length = math.hypot(candidate.x - current[0], candidate.y - current[1])
        path_source = "euclidean"
        reachable = True
        path_length = euclidean_length
        if path_lengths_m is not None and candidate.index in path_lengths_m:
            supplied = path_lengths_m[candidate.index]
            path_source = "nav2"
            reachable = supplied is not None and math.isfinite(supplied) and supplied >= 0.0
            path_length = float(supplied) if reachable else math.inf
        yaw_change = abs(wrap_angle(candidate.yaw - current[2]))
        information_gain = visibility.information_gain_nats or 0.0
        visible_term = visibility.visible_fraction
        clearance_term = float(np.clip(clearance / config.desired_clearance_m, 0.0, 1.0))
        path_penalty = min(path_length / config.max_path_length_m, 1.0)
        turn_penalty = yaw_change / math.pi
        occlusion_penalty = map_occlusion_fraction
        total = (
            config.information_gain_weight * information_gain
            + config.visible_fraction_weight * visible_term
            + config.clearance_weight * clearance_term
            - config.path_length_weight * path_penalty
            - config.turn_weight * turn_penalty
            - config.occlusion_weight * occlusion_penalty
        )
        rejection_reasons = []
        if not footprint_free:
            rejection_reasons.append("footprint_collision_or_outside_map")
        if not reachable:
            rejection_reasons.append("nav2_unreachable")
        rejection_reasons.extend(visibility.reasons)
        if rejection_reasons:
            total = -math.inf
        scores.append(
            CandidateScore(
                candidate,
                visibility,
                footprint_free,
                reachable,
                clearance,
                map_occlusion_fraction,
                path_length,
                yaw_change,
                information_gain,
                visible_term,
                clearance_term,
                path_penalty,
                turn_penalty,
                occlusion_penalty,
                total,
                tuple(dict.fromkeys(rejection_reasons)),
                path_source,
            )
        )
    return sorted(scores, key=lambda score: (-score.total_score, score.candidate.index))
