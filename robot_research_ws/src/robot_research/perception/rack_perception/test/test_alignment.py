import math

from rack_perception.alignment import calculate_alignment


def test_alignment_reports_distance_lateral_and_heading_errors():
    result = calculate_alignment(
        robot_xy=(1.0, 0.2),
        robot_yaw=0.1,
        front_center=(2.0, 0.0),
        front_normal=(-1.0, 0.0),
        rack_axis=(0.0, 1.0),
        desired_distance_m=0.65,
    )
    assert math.isclose(result.current_distance_m, 1.0)
    assert math.isclose(result.distance_error_m, 0.35)
    assert math.isclose(result.lateral_error_m, -0.2)
    assert math.isclose(result.angle_error_rad, -0.1)
