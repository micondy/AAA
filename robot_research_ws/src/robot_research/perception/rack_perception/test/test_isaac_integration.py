"""Static contract checks for the thin Isaac rack profile."""

from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[5]


def test_isaac_scene_has_three_labeled_layers_and_semantic_output():
    """Keep the simulation truth labels wired to the detector topics."""
    source = (WORKSPACE / "tools" / "isaac_sim" / "run_mobile_manipulator.py").read_text(
        encoding="utf-8"
    )
    assert "for index, height in enumerate((0.55, 1.05, 1.55), start=1)" in source
    assert "def create_rack_grid" in source
    assert 'f"rack_{rack_index:02d}_layer_{layer}"' in source
    assert '"rack_post"' in source
    assert '"SemanticPublisher.inputs:type", "semantic_segmentation"' in source
    assert '"SemanticPublisher.inputs:enableSemanticLabels", True' in source
    assert '"--add-rack"' in source


def test_rack_launch_can_optionally_start_nvblox():
    """Keep geometric mapping optional for constrained simulation GPUs."""
    source = (
        WORKSPACE
        / "src"
        / "robot_research"
        / "integration"
        / "mobile_manipulator_bringup"
        / "launch"
        / "sim_rack.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("enable_nvblox", default_value="false")' in source
    assert 'condition=IfCondition(LaunchConfiguration("enable_nvblox"))' in source
    assert '"input_qos": "DEFAULT"' in source


def test_sim_can_replace_truth_detector_with_trained_yolo():
    launch = (
        WORKSPACE / "src" / "robot_research" / "integration"
        / "mobile_manipulator_bringup" / "launch" / "sim_rack.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_yolo", default_value="false")' in launch
    assert 'condition=UnlessCondition(LaunchConfiguration("use_yolo"))' in launch
    assert '"rack_perception.yolo_detector_node"' in launch
    assert 'use_yolo:=true requires anonymous_instances:=true' in launch
