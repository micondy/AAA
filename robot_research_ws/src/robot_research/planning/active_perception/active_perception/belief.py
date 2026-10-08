"""A small, deterministic Gaussian target-position belief filter."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BeliefSnapshot:
    """Immutable copy of the current target-position belief."""

    mean: np.ndarray
    covariance: np.ndarray
    accepted_count: int
    rejected_count: int
    stamp_seconds: float


@dataclass(frozen=True)
class UpdateResult:
    """Outcome of one attempted measurement update."""

    accepted: bool
    reason: str
    mahalanobis_squared: float
    snapshot: BeliefSnapshot | None


def regularize_covariance(
    covariance: np.ndarray,
    variance_floor: float,
    variance_ceiling: float,
) -> np.ndarray:
    """Return a finite symmetric positive-definite 3-D covariance."""
    matrix = np.asarray(covariance, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("measurement covariance must be a finite 3x3 matrix")
    matrix = 0.5 * (matrix + matrix.T)
    eigenvalues, eigenvectors = np.linalg.eigh(matrix)
    eigenvalues = np.clip(eigenvalues, variance_floor, variance_ceiling)
    return eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T


class TargetBeliefFilter:
    """Fuse static-target position observations with innovation gating."""

    def __init__(
        self,
        *,
        variance_floor: float = 1.0e-6,
        variance_ceiling: float = 0.25,
        process_variance_per_second: float = 1.0e-6,
        gate_mahalanobis_squared: float = 11.344867,
    ) -> None:
        if variance_floor <= 0.0 or variance_ceiling < variance_floor:
            raise ValueError("invalid covariance limits")
        if process_variance_per_second < 0.0:
            raise ValueError("process variance must be non-negative")
        if gate_mahalanobis_squared <= 0.0:
            raise ValueError("gate must be positive")
        self.variance_floor = variance_floor
        self.variance_ceiling = variance_ceiling
        self.process_variance_per_second = process_variance_per_second
        self.gate_mahalanobis_squared = gate_mahalanobis_squared
        self._snapshot: BeliefSnapshot | None = None

    @property
    def snapshot(self) -> BeliefSnapshot | None:
        """Return a defensive copy of the current state."""
        if self._snapshot is None:
            return None
        return BeliefSnapshot(
            mean=self._snapshot.mean.copy(),
            covariance=self._snapshot.covariance.copy(),
            accepted_count=self._snapshot.accepted_count,
            rejected_count=self._snapshot.rejected_count,
            stamp_seconds=self._snapshot.stamp_seconds,
        )

    def reset(self) -> None:
        """Discard all observations."""
        self._snapshot = None

    def update(
        self,
        position: np.ndarray,
        covariance: np.ndarray,
        stamp_seconds: float,
    ) -> UpdateResult:
        """Attempt one position update and return the resulting state."""
        measurement = np.asarray(position, dtype=float)
        if measurement.shape != (3,) or not np.all(np.isfinite(measurement)):
            raise ValueError("measurement position must be a finite 3-vector")
        if not np.isfinite(stamp_seconds):
            raise ValueError("measurement timestamp must be finite")
        noise = regularize_covariance(
            covariance, self.variance_floor, self.variance_ceiling
        )

        if self._snapshot is None:
            self._snapshot = BeliefSnapshot(
                mean=measurement.copy(),
                covariance=noise,
                accepted_count=1,
                rejected_count=0,
                stamp_seconds=stamp_seconds,
            )
            return UpdateResult(True, "initialized", 0.0, self.snapshot)

        previous = self._snapshot
        if stamp_seconds < previous.stamp_seconds:
            rejected = BeliefSnapshot(
                mean=previous.mean,
                covariance=previous.covariance,
                accepted_count=previous.accepted_count,
                rejected_count=previous.rejected_count + 1,
                stamp_seconds=previous.stamp_seconds,
            )
            self._snapshot = rejected
            return UpdateResult(False, "out_of_order", float("nan"), self.snapshot)

        elapsed = max(0.0, stamp_seconds - previous.stamp_seconds)
        predicted_covariance = previous.covariance + np.eye(3) * (
            elapsed * self.process_variance_per_second
        )
        innovation = measurement - previous.mean
        innovation_covariance = predicted_covariance + noise
        mahalanobis_squared = float(
            innovation.T @ np.linalg.solve(innovation_covariance, innovation)
        )
        if mahalanobis_squared > self.gate_mahalanobis_squared:
            self._snapshot = BeliefSnapshot(
                mean=previous.mean,
                covariance=previous.covariance,
                accepted_count=previous.accepted_count,
                rejected_count=previous.rejected_count + 1,
                stamp_seconds=previous.stamp_seconds,
            )
            return UpdateResult(
                False, "innovation_gate", mahalanobis_squared, self.snapshot
            )

        gain = predicted_covariance @ np.linalg.inv(innovation_covariance)
        mean = previous.mean + gain @ innovation
        identity = np.eye(3)
        covariance_after = (
            (identity - gain)
            @ predicted_covariance
            @ (identity - gain).T
            + gain @ noise @ gain.T
        )
        covariance_after = regularize_covariance(
            covariance_after, self.variance_floor, self.variance_ceiling
        )
        self._snapshot = BeliefSnapshot(
            mean=mean,
            covariance=covariance_after,
            accepted_count=previous.accepted_count + 1,
            rejected_count=previous.rejected_count,
            stamp_seconds=stamp_seconds,
        )
        return UpdateResult(True, "fused", mahalanobis_squared, self.snapshot)
