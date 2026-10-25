import numpy as np

from rack_perception.geometry import fit_layer


def test_fit_layer_recovers_height_axis_and_camera_facing_front():
    xs, ys = np.meshgrid(np.linspace(1.65, 2.25, 45), np.linspace(-0.7, 0.7, 85))
    points = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 0.72)))
    estimate = fit_layer(points, camera_map=(0.0, 0.0, 1.1), source_id=2)
    assert estimate is not None
    assert abs(estimate.center[2] - 0.72) < 1.0e-6
    assert abs(estimate.axis[1]) > 0.99
    assert estimate.front_normal[0] < -0.99
    assert abs(estimate.front_center[0] - 1.65) < 0.03
    assert estimate.source_id == 2


def test_fit_layer_uses_surface_bounds_for_center_when_pixels_are_uneven():
    xs, ys = np.meshgrid(np.linspace(1.6, 2.4, 41), np.linspace(-1.0, 1.0, 101))
    uniform = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 1.05)))
    crowded = uniform[uniform[:, 0] > 2.2]
    points = np.vstack((uniform, np.repeat(crowded, 3, axis=0)))
    estimate = fit_layer(points, camera_map=(0.0, 0.0, 1.7), source_id=12)
    assert estimate is not None
    assert abs(estimate.center[0] - 2.0) < 0.06
    assert abs(estimate.center[1]) < 0.06


def test_axis_prior_keeps_partially_visible_parallel_rack_square_to_map():
    xs, ys = np.meshgrid(np.linspace(1.6, 2.4, 80), np.linspace(-1.0, 1.0, 120))
    # A diagonal occluder leaves a wedge; its PCA axis is not the bed axis.
    visible = xs < 1.75 + 0.26 * (ys + 1.0)
    points = np.column_stack((xs[visible], ys[visible], np.full(visible.sum(), 1.05)))
    free = fit_layer(points, camera_map=(0.0, 0.0, 1.7), source_id=12)
    constrained = fit_layer(
        points, camera_map=(0.0, 0.0, 1.7), source_id=12,
        axis_prior_yaw_rad=np.pi / 2,
    )
    assert free is not None and constrained is not None
    assert abs(free.axis[0]) > 0.08
    assert abs(constrained.axis[0]) < 1.0e-8
    assert abs(constrained.axis[1]) > 0.99
    edge = np.asarray(constrained.boundary)
    assert abs(edge[1, 0] - edge[0, 0]) < 1.0e-8
