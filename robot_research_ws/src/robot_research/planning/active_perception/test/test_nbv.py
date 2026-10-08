import math
from pathlib import Path
from types import SimpleNamespace

from active_perception.nbv import (
    evaluate_base_candidates,
    generate_base_candidates,
    NBVConfig,
    OccupancyGrid2D,
    target_in_candidate_camera,
)
from active_perception.nbv_planner_node import (
    effective_path_target_clearance,
    path_minimum_xy_distance,
)
from active_perception.visibility import PinholeCamera
import numpy as np
import pytest


CAMERA = PinholeCamera(640, 480, 500.0, 500.0, 320.0, 240.0)
CAMERA_FROM_BASE = np.array(
    (
        (0.0, -1.0, 0.0, 0.0),
        (0.0, 0.0, -1.0, 1.0),
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
)


def _free_grid():
    return OccupancyGrid2D(np.zeros((100, 100)), 0.1, -5.0, -5.0)


def test_candidate_count_and_facing_yaw_are_deterministic():
    candidates = generate_base_candidates((1.0, 2.0), radii_m=(0.5, 1.0), samples_per_ring=8)
    assert len(candidates) == 16
    first = candidates[0]
    assert (first.x, first.y) == pytest.approx((1.5, 2.0))
    assert abs(abs(first.yaw) - np.pi) < 1.0e-9


def test_candidate_transform_places_faced_target_on_optical_axis():
    candidate = generate_base_candidates((0.0, 0.0), radii_m=(1.0,), samples_per_ring=4)[0]
    center, _, _ = target_in_candidate_camera(
        candidate, (0.0, 0.0, 1.0), (0.2, 0.2, 0.2), CAMERA_FROM_BASE
    )
    assert center == pytest.approx((0.0, 0.0, 1.0), abs=1.0e-9)


def test_grid_rejects_collision_and_outside_map():
    data = np.zeros((20, 20))
    data[10, 10] = 100
    grid = OccupancyGrid2D(data, 0.1, -1.0, -1.0)
    assert not grid.footprint_is_free(0.05, 0.05, 0.15)
    assert not grid.footprint_is_free(-0.95, -0.95, 0.15)
    assert grid.footprint_is_free(0.6, 0.6, 0.15)


def test_grid_respects_unknown_policy():
    data = np.zeros((20, 20))
    data[10, 10] = -1
    grid = OccupancyGrid2D(data, 0.1, -1.0, -1.0)
    assert not grid.footprint_is_free(0.05, 0.05, 0.05, unknown_is_occupied=True)
    assert grid.footprint_is_free(0.05, 0.05, 0.05, unknown_is_occupied=False)


def test_ray_occlusion_detects_wall_between_camera_and_target():
    data = np.zeros((100, 100))
    data[:, 50] = 100
    grid = OccupancyGrid2D(data, 0.1, -5.0, -5.0)
    blocked = grid.ray_occlusion_fraction((-1.0, 0.0), (1.0, 0.0))
    clear = grid.ray_occlusion_fraction((-1.0, 1.0), (-0.5, 1.0))
    assert blocked > 0.0
    assert clear == 0.0


def test_free_visible_candidates_receive_finite_scores():
    candidates = generate_base_candidates((0.0, 0.0), radii_m=(1.0,), samples_per_ring=8)
    scores = evaluate_base_candidates(
        candidates,
        current_base_pose=(1.5, 0.0, np.pi),
        target_world=(0.0, 0.0, 1.0),
        target_size_world=(0.2, 0.2, 0.2),
        prior_covariance=np.eye(3) * 0.01,
        camera=CAMERA,
        camera_from_base=CAMERA_FROM_BASE,
        occupancy_grid=_free_grid(),
        config=NBVConfig(radii_m=(1.0,), samples_per_ring=8, robot_radius_m=0.2),
    )
    assert len(scores) == 8
    assert all(np.isfinite(score.total_score) for score in scores)
    assert scores[0].total_score >= scores[-1].total_score


def test_single_noisy_map_cell_is_not_promoted_to_total_occlusion():
    data = np.zeros((100, 100))
    data[50, 55] = 100
    grid = OccupancyGrid2D(data, 0.1, -5.0, -5.0)
    candidate = generate_base_candidates(
        (0.0, 0.0), radii_m=(1.0,), samples_per_ring=4
    )[0]
    score = evaluate_base_candidates(
        [candidate],
        current_base_pose=(1.5, 0.0, np.pi),
        target_world=(0.0, 0.0, 1.0),
        target_size_world=(0.2, 0.2, 0.2),
        prior_covariance=np.eye(3) * 0.01,
        camera=CAMERA,
        camera_from_base=CAMERA_FROM_BASE,
        occupancy_grid=grid,
        config=NBVConfig(radii_m=(1.0,), samples_per_ring=4, robot_radius_m=0.2),
    )[0]
    assert 0.0 < score.map_occlusion_fraction < 0.8
    assert score.visibility.visible
    assert np.isfinite(score.total_score)


def test_occupied_candidate_is_rejected():
    data = np.zeros((100, 100))
    data[50, 60] = 100
    grid = OccupancyGrid2D(data, 0.1, -5.0, -5.0)
    candidate = generate_base_candidates((0.0, 0.0), radii_m=(1.0,), samples_per_ring=4)[0]
    score = evaluate_base_candidates(
        [candidate],
        current_base_pose=(1.5, 0.0, np.pi),
        target_world=(0.0, 0.0, 1.0),
        target_size_world=(0.2, 0.2, 0.2),
        prior_covariance=np.eye(3) * 0.01,
        camera=CAMERA,
        camera_from_base=CAMERA_FROM_BASE,
        occupancy_grid=grid,
        config=NBVConfig(radii_m=(1.0,), samples_per_ring=4, robot_radius_m=0.2),
    )[0]
    assert not score.footprint_free
    assert score.total_score == -np.inf


def test_nav2_unreachable_candidate_is_rejected():
    candidate = generate_base_candidates((0.0, 0.0), radii_m=(1.0,), samples_per_ring=4)[0]
    score = evaluate_base_candidates(
        [candidate],
        current_base_pose=(1.5, 0.0, np.pi),
        target_world=(0.0, 0.0, 1.0),
        target_size_world=(0.2, 0.2, 0.2),
        prior_covariance=np.eye(3) * 0.01,
        camera=CAMERA,
        camera_from_base=CAMERA_FROM_BASE,
        occupancy_grid=_free_grid(),
        path_lengths_m={candidate.index: None},
        config=NBVConfig(radii_m=(1.0,), samples_per_ring=4, robot_radius_m=0.2),
    )[0]
    assert not score.reachable
    assert "nav2_unreachable" in score.rejection_reasons


def test_shorter_nav2_path_improves_otherwise_equal_score():
    candidates = generate_base_candidates((0.0, 0.0), radii_m=(1.0,), samples_per_ring=4)
    selected = [candidates[0], candidates[2]]
    scores = evaluate_base_candidates(
        selected,
        current_base_pose=(0.0, 2.0, -np.pi / 2.0),
        target_world=(0.0, 0.0, 1.0),
        target_size_world=(0.2, 0.2, 0.2),
        prior_covariance=np.eye(3) * 0.01,
        camera=CAMERA,
        camera_from_base=CAMERA_FROM_BASE,
        occupancy_grid=_free_grid(),
        path_lengths_m={selected[0].index: 4.0, selected[1].index: 1.0},
        config=NBVConfig(radii_m=(1.0,), samples_per_ring=4, robot_radius_m=0.2),
    )
    by_index = {score.candidate.index: score for score in scores}
    assert by_index[selected[1].index].path_penalty < by_index[selected[0].index].path_penalty


def test_invalid_nbv_config_is_rejected():
    with pytest.raises(ValueError, match="samples_per_ring"):
        NBVConfig(samples_per_ring=2).validate()


def test_nav2_candidate_queries_have_timeout_and_cancel_guard():
    package_root = Path(__file__).resolve().parents[1]
    config = (package_root / "config" / "nbv.yaml").read_text()
    node_source = (
        package_root / "active_perception" / "nbv_planner_node.py"
    ).read_text()
    assert "path_query_timeout_seconds: 8.0" in config
    assert 'create_timer(0.1, self._monitor_query_timeout)' in node_source
    assert "goal_handle.cancel_goal_async()" in node_source
    assert "marking it unreachable" in node_source


def test_preliminary_rejection_response_is_explainable():
    package_root = Path(__file__).resolve().parents[1]
    node_source = (
        package_root / "active_perception" / "nbv_planner_node.py"
    ).read_text()
    assert "no feasible preliminary candidates" in node_source
    assert "footprint_free={free_count}" in node_source
    assert "visible={visible_count}" in node_source


def test_path_target_clearance_uses_complete_path_not_only_goal():
    def stamped(x, y):
        position = SimpleNamespace(x=x, y=y)
        return SimpleNamespace(pose=SimpleNamespace(position=position))

    path = SimpleNamespace(
        poses=[stamped(0.0, 0.0), stamped(1.0, 0.1), stamped(2.0, 0.0)]
    )
    assert path_minimum_xy_distance(path, (1.0, 0.0)) == pytest.approx(0.1)
    assert path_minimum_xy_distance(SimpleNamespace(poses=[]), (1.0, 0.0)) == math.inf


def test_path_target_clearance_allows_only_safe_egress_inside_nominal_radius():
    assert effective_path_target_clearance(1.1, 0.87, 0.05) == pytest.approx(0.82)
    assert effective_path_target_clearance(1.1, 1.50, 0.05) == pytest.approx(1.1)
    with pytest.raises(ValueError, match="finite and non-negative"):
        effective_path_target_clearance(1.1, 0.87, -0.01)
