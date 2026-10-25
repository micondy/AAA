"""Transport-independent rack observation and persistent-map models."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Any


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class LayerObservation:
    source_id: int
    center: tuple[float, float, float]
    axis: tuple[float, float]
    front_center: tuple[float, float, float]
    front_normal: tuple[float, float]
    boundary: tuple[tuple[float, float, float], ...]
    confidence: float
    observed_ratio: float
    point_count: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "LayerObservation":
        return cls(
            source_id=int(value["source_id"]),
            center=tuple(float(v) for v in value["center"]),
            axis=tuple(float(v) for v in value["axis"]),
            front_center=tuple(float(v) for v in value["front_center"]),
            front_normal=tuple(float(v) for v in value["front_normal"]),
            boundary=tuple(tuple(float(v) for v in p) for p in value["boundary"]),
            confidence=float(value["confidence"]),
            observed_ratio=float(value["observed_ratio"]),
            point_count=int(value["point_count"]),
        )


def encode_observations(
    observations: list[LayerObservation], frame_id: str, stamp_s: float
) -> str:
    return json.dumps(
        {
            "schema": "rack_observations/v1",
            "frame_id": frame_id,
            "stamp_s": float(stamp_s),
            "layers": [item.to_dict() for item in observations],
        },
        separators=(",", ":"),
    )


def decode_observations(payload: str) -> tuple[str, float, list[LayerObservation]]:
    value = json.loads(payload)
    if value.get("schema") != "rack_observations/v1":
        raise ValueError("unsupported rack observation schema")
    return (
        str(value["frame_id"]),
        float(value["stamp_s"]),
        [LayerObservation.from_dict(item) for item in value.get("layers", [])],
    )
