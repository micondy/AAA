from active_perception.visibility import (
    estimate_depth_occlusion,
    evaluate_view,
    gaussian_information_gain,
    PinholeCamera,
    project_axis_aligned_box,
    VisibilityConfig,
)
import numpy as np
import pytest


CAMERA = PinholeCamera(width=100, height=80, fx=100.0, fy=100.0, cx=50.0, cy=40.0)


def test_centered_target_is_visible_and_has_finite_projection():
    result = evaluate_view(CAMERA, (0.0, 0.0, 2.0), (0.4, 0.4, 0.2))
    assert result.visible
    assert result.frustum_fraction == pytest.approx(1.0)
    assert result.projected_area_pixels > 0.0
    assert result.projected_area_ratio > 0.0
    assert np.all(np.diag(result.expected_covariance) > 0.0)


def test_target_outside_image_is_rejected():
    result = evaluate_view(CAMERA, (3.0, 0.0, 2.0), (0.2, 0.2, 0.2))
    assert not result.visible
    assert "outside_frustum_or_near_plane" in result.reasons


def test_target_intersecting_near_plane_has_no_projection():
    projection = project_axis_aligned_box(
        CAMERA, (0.0, 0.0, 0.15), (0.1, 0.1, 0.2), near_plane_m=0.1
    )
    assert projection is None


def test_partial_projection_reports_fraction_between_zero_and_one():
    result = evaluate_view(CAMERA, (0.9, 0.0, 2.0), (0.8, 0.4, 0.2))
    assert result.projection is not None
    assert 0.0 < result.frustum_fraction < 1.0


def test_depth_occlusion_detects_foreground_pixels():
    projection = project_axis_aligned_box(CAMERA, (0.0, 0.0, 2.0), (0.4, 0.4, 0.2))
    assert projection is not None
    depth = np.full((80, 100), 2.0)
    u0 = int(projection.clipped_min_u)
    u1 = int(projection.clipped_max_u)
    v0 = int(projection.clipped_min_v)
    v1 = int(projection.clipped_max_v)
    depth[v0:v1, u0:(u0 + u1) // 2] = 1.0
    occlusion = estimate_depth_occlusion(depth, projection)
    assert occlusion.known
    assert 0.4 < occlusion.fraction < 0.6


def test_invalid_depth_does_not_claim_known_occlusion():
    projection = project_axis_aligned_box(CAMERA, (0.0, 0.0, 2.0), (0.4, 0.4, 0.2))
    assert projection is not None
    occlusion = estimate_depth_occlusion(np.zeros((80, 100)), projection)
    assert not occlusion.known
    assert occlusion.fraction == 0.0


def test_heavy_occlusion_rejects_view_and_inflates_covariance():
    clear = evaluate_view(CAMERA, (0.0, 0.0, 2.0), (0.4, 0.4, 0.2))
    depth = np.full((80, 100), 1.0)
    blocked = evaluate_view(
        CAMERA, (0.0, 0.0, 2.0), (0.4, 0.4, 0.2), depth_m=depth
    )
    assert not blocked.visible
    assert "occluded" in blocked.reasons
    assert np.trace(blocked.expected_covariance) > np.trace(clear.expected_covariance)


def test_external_map_occlusion_can_reject_hypothetical_view():
    result = evaluate_view(
        CAMERA,
        (0.0, 0.0, 2.0),
        (0.4, 0.4, 0.2),
        external_occlusion_fraction=1.0,
    )
    assert not result.visible
    assert result.occlusion_known
    assert result.occlusion_fraction == pytest.approx(1.0)


def test_surface_normal_distinguishes_front_and_back_views():
    front = evaluate_view(
        CAMERA,
        (0.0, 0.0, 2.0),
        (0.4, 0.4, 0.2),
        surface_normal_camera=(0.0, 0.0, -1.0),
    )
    back = evaluate_view(
        CAMERA,
        (0.0, 0.0, 2.0),
        (0.4, 0.4, 0.2),
        surface_normal_camera=(0.0, 0.0, 1.0),
    )
    assert front.incidence_cosine == pytest.approx(1.0)
    assert front.visible
    assert back.incidence_cosine == pytest.approx(0.0)
    assert not back.visible


def test_expected_covariance_grows_with_distance():
    near = evaluate_view(CAMERA, (0.0, 0.0, 1.0), (0.3, 0.3, 0.2))
    far = evaluate_view(CAMERA, (0.0, 0.0, 3.0), (0.3, 0.3, 0.2))
    assert np.trace(far.expected_covariance) > np.trace(near.expected_covariance)


def test_information_gain_is_positive_and_lower_for_noisy_measurement():
    prior = np.eye(3) * 0.02
    precise = gaussian_information_gain(prior, np.eye(3) * 0.001)
    noisy = gaussian_information_gain(prior, np.eye(3) * 0.1)
    assert precise > noisy > 0.0


def test_evaluate_view_reports_information_gain_for_visible_target():
    result = evaluate_view(
        CAMERA,
        (0.0, 0.0, 2.0),
        (0.4, 0.4, 0.2),
        prior_covariance=np.eye(3) * 0.01,
    )
    assert result.information_gain_nats is not None
    assert result.information_gain_nats > 0.0


def test_invalid_geometry_and_covariance_are_rejected():
    with pytest.raises(ValueError, match="size_xyz"):
        evaluate_view(CAMERA, (0.0, 0.0, 2.0), (0.0, 0.2, 0.2))
    with pytest.raises(ValueError, match="positive definite"):
        gaussian_information_gain(np.zeros((3, 3)), np.eye(3))


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError, match="max_distance"):
        VisibilityConfig(min_distance_m=2.0, max_distance_m=1.0).validate()
