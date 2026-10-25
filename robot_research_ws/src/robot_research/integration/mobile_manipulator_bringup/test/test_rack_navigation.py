"""Static safety checks for the combined rack simulation profile."""

import ast
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[5]
BRINGUP = Path(__file__).resolve().parents[1]
NAVIGATION = ROOT / "src/robot_research/navigation/mobile_navigation_bringup"
SIMULATOR = ROOT / "tools/isaac_sim/run_mobile_manipulator.py"


def test_rack_profile_routes_web_and_nav2_through_one_collision_monitor():
    source = (BRINGUP / "launch/sim_rack_navigation.launch.py").read_text()
    navigation = (NAVIGATION / "launch/navigation.launch.py").read_text()

    assert '"params_file": str(navigation_share / "config" / "nav2.yaml")' in source
    assert '"cmd_vel_smoothed_topic": "/cmd_vel_nav_smoothed"' in source
    assert '"nav_input_topic": "/cmd_vel_nav_smoothed"' in source
    assert '"manual_input_topic": "/cmd_vel_web"' in source
    assert '"output_topic": "/cmd_vel_muxed"' in source
    assert '"cmd_vel_in_topic": "/cmd_vel_muxed"' in source
    assert '"cmd_vel_out_topic": "/cmd_vel"' in source
    assert '"cmd_vel_in_topic": LaunchConfiguration("cmd_vel_in_topic")' in navigation
    assert '"xy_goal_tolerance": "0.08"' in source
    assert '"fresh_map": "false"' in source


def test_rack_collision_sensors_are_not_disabled():
    overlay = yaml.safe_load((NAVIGATION / "config/costmap_rack_grid_overlay.yaml").read_text())
    monitor = overlay["collision_monitor"]["ros__parameters"]
    assert monitor["FrontStop"]["enabled"] is True
    assert monitor["front_depth"]["enabled"] is True
    for key in ("local_costmap", "global_costmap"):
        costmap = overlay[key][key]["ros__parameters"]
        assert costmap["inflation_layer"]["inflation_radius"] >= 0.33


def test_truth_world_frame_does_not_compete_with_map_for_odom_parent():
    source = SIMULATOR.read_text()
    assert '("WorldOdomTfPub.inputs:parentFrameId", "odom")' in source
    assert '("WorldOdomTfPub.inputs:childFrameId", "sim_world")' in source
    assert '("WorldOdomTfPub.inputs:parentFrameId", "sim_world")' not in source

    # Import only the pure transform function; importing the whole file starts Isaac.
    parsed = ast.parse(source)
    function = next(
        node for node in parsed.body
        if isinstance(node, ast.FunctionDef) and node.name == "inverse_transform"
    )
    namespace = {"math": math}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SIMULATOR), "exec"), namespace)
    inverse = namespace["inverse_transform"]
    translation, rotation = inverse((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    assert translation == [-1.0, -2.0, -3.0]
    assert rotation == [-0.0, -0.0, -0.0, 1.0]
    half_turn = math.sqrt(0.5)
    translation, rotation = inverse((1.0, 0.0, 0.0), (0.0, 0.0, half_turn, half_turn))
    assert math.isclose(translation[0], 0.0, abs_tol=1e-12)
    assert math.isclose(translation[1], 1.0, abs_tol=1e-12)
    assert math.isclose(rotation[2], -half_turn, abs_tol=1e-12)
