from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_real_profile_keeps_instance_ids_frame_local():
    with (PACKAGE / "config" / "rack_real_yolo.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    assert config["rack_map"]["ros__parameters"]["use_source_instance_hints"] is False
    assert config["rack_geometry"]["ros__parameters"]["filter_expected_heights"] is False
    assert config["rack_detector"]["ros__parameters"]["color_topic"] == "/sensors/nav_camera/color/image_raw"


def test_real_launch_does_not_start_navigation_or_isaac():
    launch = (PACKAGE / "launch" / "rack_real_yolo.launch.py").read_text(encoding="utf-8")
    assert "rack_perception.yolo_detector_node" in launch
    assert "mobile_navigation_bringup" not in launch
    assert "sim_rack" not in launch
