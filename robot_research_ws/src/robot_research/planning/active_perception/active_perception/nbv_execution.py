"""Pure helpers for recording a next-best-view execution experiment."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BeliefSnapshot:
    """Small serializable snapshot used before and after one NBV motion."""

    observation_count: int
    covariance_trace: float
    position_xyz: tuple[float, float, float]
    visible: bool


def make_belief_snapshot(message, observation_count: int, visible: bool) -> BeliefSnapshot:
    """Extract position and XYZ covariance trace from a ROS belief message."""
    covariance = message.pose.covariance
    trace = float(covariance[0] + covariance[7] + covariance[14])
    position = message.pose.pose.position
    if not math.isfinite(trace) or trace < 0.0:
        raise ValueError("belief covariance trace must be finite and non-negative")
    return BeliefSnapshot(
        observation_count=int(observation_count),
        covariance_trace=trace,
        position_xyz=(float(position.x), float(position.y), float(position.z)),
        visible=bool(visible),
    )


def summarize_belief_change(before: BeliefSnapshot, after: BeliefSnapshot) -> dict:
    """Return interpretable paired metrics without claiming ground-truth accuracy."""
    displacement = float(
        np.linalg.norm(np.asarray(after.position_xyz) - np.asarray(before.position_xyz))
    )
    reduction = before.covariance_trace - after.covariance_trace
    ratio = (
        reduction / before.covariance_trace
        if before.covariance_trace > 0.0
        else math.nan
    )
    return {
        "before": asdict(before),
        "after": asdict(after),
        "new_observations": after.observation_count - before.observation_count,
        "covariance_trace_reduction": reduction,
        "covariance_trace_reduction_ratio": ratio,
        "belief_position_shift_m": displacement,
    }
