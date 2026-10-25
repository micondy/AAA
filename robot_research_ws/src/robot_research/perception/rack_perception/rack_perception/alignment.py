"""Pure calculations for final rack-relative alignment."""

from __future__ import annotations

from dataclasses import dataclass
import math

from rack_perception.model import normalize_angle


@dataclass(frozen=True)
class AlignmentError:
    current_distance_m: float
    distance_error_m: float
    lateral_error_m: float
    angle_error_rad: float


def calculate_alignment(
    robot_xy: tuple[float, float],
    robot_yaw: float,
    front_center: tuple[float, float],
    front_normal: tuple[float, float],
    rack_axis: tuple[float, float],
    desired_distance_m: float,
) -> AlignmentError:
    delta = (robot_xy[0] - front_center[0], robot_xy[1] - front_center[1])
    current_distance = delta[0] * front_normal[0] + delta[1] * front_normal[1]
    lateral = -(delta[0] * rack_axis[0] + delta[1] * rack_axis[1])
    desired_yaw = math.atan2(-front_normal[1], -front_normal[0])
    return AlignmentError(
        current_distance_m=current_distance,
        distance_error_m=current_distance - float(desired_distance_m),
        lateral_error_m=lateral,
        angle_error_rad=normalize_angle(desired_yaw - robot_yaw),
    )
