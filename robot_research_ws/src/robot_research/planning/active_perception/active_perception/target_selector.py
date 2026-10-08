"""Deterministic task-target selection from stable tracked detections."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TargetCandidate:
    """Minimal tracked-object data needed by the selector."""

    object_id: str
    class_id: str
    score: float
    position: tuple[float, float, float]
    source_index: int = 0
    result_index: int = 0


@dataclass(frozen=True)
class SelectionResult:
    """Selector state after processing one detection frame."""

    selected_id: str | None
    candidate: TargetCandidate | None
    visible: bool
    selection_changed: bool
    state: str
    missed_frames: int


class TargetSelector:
    """Lock one stable ID and avoid switching during short observation gaps."""

    def __init__(
        self,
        target_id: str = "",
        target_class: str = "*",
        min_score: float = 0.0,
        max_missed_frames: int = 5,
    ) -> None:
        self.selected_id: str | None = None
        self.missed_frames = 0
        self.configure(target_id, target_class, min_score, max_missed_frames)

    def configure(
        self,
        target_id: str,
        target_class: str,
        min_score: float,
        max_missed_frames: int,
    ) -> None:
        """Apply selection criteria, clearing a lock when criteria change."""
        target_id = target_id.strip()
        target_class = target_class.strip() or "*"
        if not math.isfinite(min_score) or not 0.0 <= min_score <= 1.0:
            raise ValueError("min_score must be finite and in [0, 1]")
        if max_missed_frames < 0:
            raise ValueError("max_missed_frames must be non-negative")
        new_config = (
            target_id,
            target_class,
            float(min_score),
            int(max_missed_frames),
        )
        if getattr(self, "_config", None) != new_config:
            self.selected_id = None
            self.missed_frames = 0
        self.target_id, self.target_class, self.min_score, self.max_missed_frames = (
            new_config
        )
        self._config = new_config

    @staticmethod
    def _validate(candidate: TargetCandidate) -> None:
        if not candidate.object_id:
            raise ValueError("candidate object_id must not be empty")
        if not candidate.class_id:
            raise ValueError("candidate class_id must not be empty")
        if not math.isfinite(candidate.score):
            raise ValueError("candidate score must be finite")
        if len(candidate.position) != 3 or not all(
            math.isfinite(value) for value in candidate.position
        ):
            raise ValueError("candidate position must contain three finite values")

    def _eligible(self, candidate: TargetCandidate) -> bool:
        if candidate.score < self.min_score:
            return False
        if self.target_id and candidate.object_id != self.target_id:
            return False
        return self.target_class == "*" or candidate.class_id == self.target_class

    def update(self, candidates: list[TargetCandidate]) -> SelectionResult:
        """Update the lock using one frame of tracked candidates."""
        for candidate in candidates:
            self._validate(candidate)
        eligible = [candidate for candidate in candidates if self._eligible(candidate)]

        if self.selected_id is not None:
            selected = next(
                (
                    candidate
                    for candidate in eligible
                    if candidate.object_id == self.selected_id
                ),
                None,
            )
            if selected is not None:
                self.missed_frames = 0
                return SelectionResult(
                    self.selected_id, selected, True, False, "tracking", 0
                )
            self.missed_frames += 1
            if self.missed_frames <= self.max_missed_frames:
                return SelectionResult(
                    self.selected_id,
                    None,
                    False,
                    False,
                    "temporarily_lost",
                    self.missed_frames,
                )
            self.selected_id = None
            self.missed_frames = 0

        if not eligible:
            return SelectionResult(None, None, False, False, "unavailable", 0)

        selected = min(
            eligible,
            key=lambda candidate: (
                -candidate.score,
                sum(value * value for value in candidate.position),
                candidate.object_id,
            ),
        )
        self.selected_id = selected.object_id
        return SelectionResult(
            self.selected_id, selected, True, True, "acquired", 0
        )
