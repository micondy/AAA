from pathlib import Path

import pytest

from rtabmap_bringup.configuration import (
    as_bool,
    camera_prefix,
    load_flat_yaml,
    mode_parameters,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_configuration_files_are_flat_dictionaries():
    odometry = load_flat_yaml(PACKAGE_ROOT / "config" / "rgbd_odometry.yaml")
    slam = load_flat_yaml(PACKAGE_ROOT / "config" / "rtabmap.yaml")

    assert odometry["subscribe_rgbd"] is True
    assert odometry["wait_imu_to_init"] is False
    assert slam["RGBD/CreateOccupancyGrid"] == "true"
    assert slam["Grid/3D"] == "false"
    assert slam["Mem/UseOdomFeatures"] == "true"
    assert odometry["Reg/Force3DoF"] == "true"
    assert slam["Reg/Force3DoF"] == "true"
    assert slam["Optimizer/Slam2D"] == "true"


def test_mapping_and_localization_memory_modes_are_distinct():
    assert mode_parameters("mapping")["Mem/IncrementalMemory"] == "true"
    assert mode_parameters("localization")["Mem/IncrementalMemory"] == "false"
    assert mode_parameters("localization")["Mem/InitWMWithAllNodes"] == "true"


def test_camera_prefix_supports_nested_namespace():
    assert camera_prefix("/robot1/sensors/", "front") == "/robot1/sensors/front"


def test_invalid_camera_prefix_is_rejected():
    with pytest.raises(ValueError):
        camera_prefix("robot-1", "front")
    with pytest.raises(ValueError):
        camera_prefix("robot1", "front/camera")


def test_invalid_boolean_is_rejected():
    assert as_bool("YES") is True
    with pytest.raises(ValueError):
        as_bool("sometimes")


def test_optional_initial_pose_is_only_forwarded_when_nonempty():
    online = (PACKAGE_ROOT / "launch" / "online.launch.py").read_text(
        encoding="utf-8"
    )
    localization = (PACKAGE_ROOT / "launch" / "localization.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'initial_pose = LaunchConfiguration("initial_pose")' in online
    assert 'if initial_pose:' in online
    assert 'slam_parameters["initial_pose"] = initial_pose' in online
    assert '"initial_pose": LaunchConfiguration("initial_pose")' in localization


def test_external_odometry_mode_disables_visual_odometry_owner():
    online = (PACKAGE_ROOT / "launch" / "online.launch.py").read_text(
        encoding="utf-8"
    )
    localization = (PACKAGE_ROOT / "launch" / "localization.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'odom_source not in {"rgbd", "external"}' in online
    assert 'if odom_source == "rgbd":' in online
    assert 'slam_parameters["subscribe_odom_info"] = False' in online
    assert 'slam_parameters["Mem/UseOdomFeatures"] = "false"' in online
    assert 'else "/rtabmap/rgbd_image"' in online
    assert 'else odom_topic' in online
    assert '"odom_source": LaunchConfiguration("odom_source")' in localization
    assert '"odom_topic": LaunchConfiguration("odom_topic")' in localization


def test_mobile_base_online_profile_uses_live_camera_and_wheel_odometry():
    mobile = (PACKAGE_ROOT / "launch" / "mobile_base_online.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"start_camera": "true"' in mobile
    assert '"odom_source": "external"' in mobile
    assert '"odom_topic": "/swerve_controller/odom"' in mobile
    assert '"fresh_map": LaunchConfiguration("fresh_map")' in mobile
    assert 'default_value="false"' in mobile
    assert '"--frame-id", "base_link"' in mobile
    assert '"--child-frame-id", "nav_camera_link"' in mobile
