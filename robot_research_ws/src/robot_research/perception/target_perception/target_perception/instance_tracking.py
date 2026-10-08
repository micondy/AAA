"""Deterministic class-aware 3D association for baseline instance IDs."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class InstanceObservation:
    """Minimal observation needed for frame-to-frame association."""

    class_id: str
    position: tuple[float, float, float]


@dataclass
class InstanceTrack:
    """One active track in the association baseline."""

    track_id: str
    class_id: str
    position: tuple[float, float, float]
    hits: int = 1
    missed_frames: int = 0


class ClassAware3DTracker:
    """Assign stable IDs using global greedy, class-gated 3D proximity."""

    def __init__(self, max_distance_m: float = 0.35, max_missed_frames: int = 5):
        if max_distance_m <= 0.0:
            raise ValueError("max_distance_m must be positive")
        if max_missed_frames < 0:
            raise ValueError("max_missed_frames must be non-negative")
        self.max_distance_m = float(max_distance_m)
        self.max_missed_frames = int(max_missed_frames)
        self.tracks: dict[str, InstanceTrack] = {}
        self._class_counters: dict[str, int] = {}

    @staticmethod
    def _distance(
        first: tuple[float, float, float],
        second: tuple[float, float, float],
    ) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))

    @staticmethod
    def _validate(observation: InstanceObservation) -> None:
        if not observation.class_id:
            raise ValueError("observation class_id must not be empty")
        if len(observation.position) != 3 or not all(
            math.isfinite(value) for value in observation.position
        ):
            raise ValueError("observation position must contain three finite values")

    def _new_track(self, observation: InstanceObservation) -> InstanceTrack:
        count = self._class_counters.get(observation.class_id, 0) + 1
        self._class_counters[observation.class_id] = count
        track = InstanceTrack(
            track_id=f"{observation.class_id}_{count:04d}",
            class_id=observation.class_id,
            position=observation.position,
        )
        self.tracks[track.track_id] = track
        return track

    def update(self, observations: list[InstanceObservation]) -> list[str]:
        """Update active tracks and return one stable ID per input observation."""
        for observation in observations:
            self._validate(observation)
        existing_track_ids = set(self.tracks)

        candidates: list[tuple[float, str, int]] = []
        for index, observation in enumerate(observations):
            for track_id, track in self.tracks.items():
                if observation.class_id != track.class_id:
                    continue
                distance = self._distance(observation.position, track.position)
                if distance <= self.max_distance_m:
                    candidates.append((distance, track_id, index))

        assignments: dict[int, str] = {}
        used_tracks: set[str] = set()
        for _, track_id, observation_index in sorted(candidates):
            if observation_index in assignments or track_id in used_tracks:
                continue
            assignments[observation_index] = track_id
            used_tracks.add(track_id)

        output_ids = []
        for index, observation in enumerate(observations):
            track_id = assignments.get(index)
            if track_id is None:
                track = self._new_track(observation)
            else:
                track = self.tracks[track_id]
                track.position = observation.position
                track.hits += 1
                track.missed_frames = 0
            output_ids.append(track.track_id)

        for track_id in existing_track_ids - used_tracks:
            self.tracks[track_id].missed_frames += 1
        expired = [
            track_id
            for track_id, track in self.tracks.items()
            if track.missed_frames > self.max_missed_frames
        ]
        for track_id in expired:
            del self.tracks[track_id]
        return output_ids
