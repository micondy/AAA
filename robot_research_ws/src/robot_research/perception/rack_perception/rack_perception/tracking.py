"""Persistent rack/layer association independent of ROS transport."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
from scipy.optimize import linear_sum_assignment

from rack_perception.model import LayerObservation


@dataclass
class LayerTrack:
    stable_id: str
    source_id: int
    center: np.ndarray
    axis: np.ndarray
    front_center: np.ndarray
    front_normal: np.ndarray
    boundary: np.ndarray
    confidence: float
    observed_ratio: float
    last_seen_s: float
    hits: int = 1
    observed: bool = True
    level_number: int = 0


@dataclass
class RackTrack:
    stable_id: str
    source_key: int | None = None
    layers: dict[str, LayerTrack] = field(default_factory=dict)
    confirmed: bool = False
    seen_frames: int = 0

    @property
    def center_xy(self) -> np.ndarray:
        if not self.layers:
            return np.zeros(2)
        return np.mean([layer.center[:2] for layer in self.layers.values()], axis=0)


class RackTracker:
    """Associate multi-view observations while retaining temporarily hidden layers."""

    def __init__(
        self,
        rack_distance_m: float = 0.85,
        group_distance_m: float = 0.85,
        layer_xy_distance_m: float = 0.65,
        layer_height_distance_m: float = 0.18,
        smoothing: float = 0.25,
        min_confirmed_layers: int = 3,
        min_confirmed_frames: int = 1,
        use_source_instance_hints: bool = True,
    ) -> None:
        self.rack_distance_m = float(rack_distance_m)
        self.group_distance_m = float(group_distance_m)
        self.layer_xy_distance_m = float(layer_xy_distance_m)
        self.layer_height_distance_m = float(layer_height_distance_m)
        self.smoothing = float(smoothing)
        self.min_confirmed_layers = int(min_confirmed_layers)
        self.min_confirmed_frames = int(min_confirmed_frames)
        self.use_source_instance_hints = bool(use_source_instance_hints)
        self.racks: dict[str, RackTrack] = {}
        self._rack_counter = 0
        self._layer_counter = 0

    def _new_rack(self, source_key: int | None = None) -> RackTrack:
        self._rack_counter += 1
        rack = RackTrack(f"rack_{self._rack_counter:04d}", source_key=source_key)
        self.racks[rack.stable_id] = rack
        return rack

    def _new_layer(
        self, rack: RackTrack, observation: LayerObservation, stamp_s: float
    ) -> LayerTrack:
        self._layer_counter += 1
        track = LayerTrack(
            stable_id=f"layer_{self._layer_counter:04d}",
            source_id=observation.source_id,
            center=np.asarray(observation.center, dtype=float),
            axis=np.asarray(observation.axis, dtype=float),
            front_center=np.asarray(observation.front_center, dtype=float),
            front_normal=np.asarray(observation.front_normal, dtype=float),
            boundary=np.asarray(observation.boundary, dtype=float),
            confidence=observation.confidence,
            observed_ratio=observation.observed_ratio,
            last_seen_s=stamp_s,
        )
        rack.layers[track.stable_id] = track
        return track

    @staticmethod
    def _groups(
        observations: list[LayerObservation], threshold: float, use_source_hints: bool
    ) -> list[list[LayerObservation]]:
        pending = list(observations)
        groups: list[list[LayerObservation]] = []
        while pending:
            group = [pending.pop(0)]
            changed = True
            while changed:
                changed = False
                for candidate in pending[:]:
                    same_source = (
                        use_source_hints
                        and candidate.source_id >= 10
                        and group[0].source_id >= 10
                        and candidate.source_id // 10 == group[0].source_id // 10
                    )
                    group_center = np.mean([member.center[:2] for member in group], axis=0)
                    nearby_distinct_layer = (
                        math.dist(candidate.center[:2], group_center) <= threshold
                        and all(abs(candidate.center[2] - member.center[2]) > 0.20 for member in group)
                    )
                    fallback_geometry = not use_source_hints or candidate.source_id < 10 or group[0].source_id < 10
                    if same_source or (fallback_geometry and nearby_distinct_layer):
                        group.append(candidate)
                        pending.remove(candidate)
                        changed = True
            groups.append(group)
        return groups

    def _anonymous_matches(
        self, groups: list[list[LayerObservation]]
    ) -> tuple[dict[int, RackTrack], set[int]]:
        """One-to-one map-landmark association without inspecting mask IDs."""
        tracks = list(self.racks.values())
        if not groups or not tracks:
            return {}, set()
        costs = np.full((len(groups), len(tracks)), np.inf)
        for i, group in enumerate(groups):
            center = np.mean([item.center[:2] for item in group], axis=0)
            axis = np.mean([item.axis for item in group], axis=0)
            axis /= max(float(np.linalg.norm(axis)), 1e-9)
            for j, track in enumerate(tracks):
                distance = float(np.linalg.norm(center - track.center_xy))
                if distance > self.rack_distance_m:
                    continue
                track_axis = np.mean([item.axis for item in track.layers.values()], axis=0)
                track_axis /= max(float(np.linalg.norm(track_axis)), 1e-9)
                axis_difference = 1.0 - abs(float(np.dot(axis, track_axis)))
                # A partly seen tray can have a badly rotated PCA axis. Keep
                # orientation as a weak preference, never as a hard veto.
                z_error = max(
                    min(abs(item.center[2] - layer.center[2]) for layer in track.layers.values())
                    for item in group
                )
                if z_error > self.layer_height_distance_m:
                    continue
                costs[i, j] = (
                    distance / self.rack_distance_m
                    + 0.15 * axis_difference
                    + 0.25 * z_error / self.layer_height_distance_m
                    - (0.05 if track.confirmed else 0.0)
                )
        # Unmatched dummy columns allow new landmarks. Existing candidates that
        # lose a one-to-one conflict are held, not duplicated immediately.
        padded = np.full((len(groups), len(tracks) + len(groups)), 1e6)
        padded[:, :len(tracks)] = np.where(np.isfinite(costs), costs, 1e6)
        for i in range(len(groups)):
            padded[i, len(tracks) + i] = 1.5
        rows, cols = linear_sum_assignment(padded)
        matches = {}
        hold = set()
        for i, j in zip(rows, cols):
            valid = sorted(value for value in costs[i] if np.isfinite(value))
            if len(valid) >= 2 and valid[1] - valid[0] < 0.10:
                hold.add(int(i))
            elif j < len(tracks) and np.isfinite(costs[i, j]):
                matches[int(i)] = tracks[int(j)]
            elif valid:
                hold.add(int(i))
        return matches, hold

    def update(self, observations: list[LayerObservation], stamp_s: float) -> None:
        for rack in self.racks.values():
            for layer in rack.layers.values():
                layer.observed = False
        groups = self._groups(observations, self.group_distance_m, self.use_source_instance_hints)
        anonymous_matches, held_groups = (
            ({}, set()) if self.use_source_instance_hints else self._anonymous_matches(groups)
        )
        for group_index, group in enumerate(groups):
            if group_index in held_groups:
                continue
            group_center = np.mean([item.center[:2] for item in group], axis=0)
            source_key = (
                group[0].source_id // 10
                if self.use_source_instance_hints and group[0].source_id >= 10 else None
            )
            if self.use_source_instance_hints:
                candidates = sorted(
                    (float(np.linalg.norm(group_center - rack.center_xy)), rack.stable_id)
                    for rack in self.racks.values() if rack.source_key == source_key
                )
                rack = (
                    self.racks[candidates[0][1]]
                    if candidates and (source_key is not None or candidates[0][0] <= self.rack_distance_m)
                    else self._new_rack(source_key)
                )
            else:
                rack = anonymous_matches.get(group_index) or self._new_rack()
            rack.seen_frames += 1
            used: set[str] = set()
            for observation in sorted(group, key=lambda item: item.center[2]):
                matches = []
                same_source_seen = False
                for layer in rack.layers.values():
                    if layer.stable_id in used:
                        continue
                    if self.use_source_instance_hints and source_key is not None and layer.source_id != observation.source_id:
                        continue
                    same_source_seen = self.use_source_instance_hints and source_key is not None
                    xy_error = float(np.linalg.norm(layer.center[:2] - observation.center[:2]))
                    z_error = abs(float(layer.center[2] - observation.center[2]))
                    if (
                        (self.use_source_instance_hints and source_key is not None or not self.use_source_instance_hints or xy_error <= self.layer_xy_distance_m)
                        and z_error <= self.layer_height_distance_m
                    ):
                        matches.append((z_error + 0.25 * xy_error, layer.stable_id))
                if not matches:
                    if same_source_seen:
                        continue
                    layer = self._new_layer(rack, observation, stamp_s)
                else:
                    layer = rack.layers[min(matches)[1]]
                    alpha = self.smoothing
                    axis = np.asarray(observation.axis, dtype=float)
                    normal = np.asarray(observation.front_normal, dtype=float)
                    if float(axis @ layer.axis) < 0.0:
                        axis = -axis
                    if float(normal @ layer.front_normal) < 0.0:
                        normal = -normal
                        observed_front = 2.0 * np.asarray(observation.center) - np.asarray(
                            observation.front_center
                        )
                    else:
                        observed_front = np.asarray(observation.front_center)
                    observed_center = np.asarray(observation.center, dtype=float)
                    observed_boundary = np.asarray(observation.boundary, dtype=float)
                    if observation.observed_ratio >= layer.observed_ratio - 0.05:
                        layer.center = (1 - alpha) * layer.center + alpha * observed_center
                        layer.front_center = (
                            (1 - alpha) * layer.front_center + alpha * observed_front
                        )
                        layer.axis = (1 - alpha) * layer.axis + alpha * axis
                        layer.axis /= np.linalg.norm(layer.axis)
                        layer.front_normal = (1 - alpha) * layer.front_normal + alpha * normal
                        layer.front_normal /= np.linalg.norm(layer.front_normal)
                        layer.boundary = (1 - alpha) * layer.boundary + alpha * observed_boundary
                    layer.confidence = (1 - alpha) * layer.confidence + alpha * observation.confidence
                    layer.observed_ratio = max(layer.observed_ratio, observation.observed_ratio)
                    layer.last_seen_s = stamp_s
                    layer.hits += 1
                    layer.observed = True
                used.add(layer.stable_id)
            if len(rack.layers) >= self.min_confirmed_layers and rack.seen_frames >= self.min_confirmed_frames:
                rack.confirmed = True
        for rack in self.racks.values():
            for number, layer in enumerate(
                sorted(rack.layers.values(), key=lambda item: item.center[2]), start=1
            ):
                layer.level_number = number

    def to_dict(self, frame_id: str, stamp_s: float) -> dict[str, Any]:
        racks = []
        for rack in sorted(self.racks.values(), key=lambda item: item.stable_id):
            if not rack.confirmed:
                continue
            layers = []
            for layer in sorted(rack.layers.values(), key=lambda item: item.level_number):
                layers.append(
                    {
                        "stable_id": layer.stable_id,
                        "source_id": layer.source_id,
                        "level_number": layer.level_number,
                        "height_m": float(layer.center[2]),
                        "center": layer.center.tolist(),
                        "axis": layer.axis.tolist(),
                        "front_center": layer.front_center.tolist(),
                        "front_normal": layer.front_normal.tolist(),
                        "boundary": layer.boundary.tolist(),
                        "confidence": float(layer.confidence),
                        "last_observed_s": float(layer.last_seen_s),
                        "observed": bool(layer.observed),
                        "observed_ratio": float(layer.observed_ratio),
                        "estimated_ratio": float(1.0 - layer.observed_ratio),
                        "observation_count": int(layer.hits),
                    }
                )
            if layers:
                axis = np.mean([np.asarray(item["axis"]) for item in layers], axis=0)
                axis /= np.linalg.norm(axis)
                center = np.mean([np.asarray(item["center"]) for item in layers], axis=0)
                racks.append(
                    {
                        "stable_id": rack.stable_id,
                        "source_instance_id": rack.source_key,
                        "position": center.tolist(),
                        "yaw_rad": float(math.atan2(axis[1], axis[0])),
                        "layers": layers,
                    }
                )
        return {
            "schema": "rack_map/v1",
            "frame_id": frame_id,
            "stamp_s": float(stamp_s),
            "racks": racks,
        }
