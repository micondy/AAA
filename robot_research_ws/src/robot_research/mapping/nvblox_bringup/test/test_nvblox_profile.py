from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _parameters():
    with (PACKAGE_ROOT / "config" / "mapping.yaml").open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)["/**"]["ros__parameters"]


def test_mapping_profile_is_bounded_and_single_camera():
    params = _parameters()
    assert params["mapping_type"] == "static_tsdf"
    assert params["num_cameras"] == 1
    assert params["global_frame"] == "odom"
    assert params["map_clearing_radius_m"] > 0.0
    assert 0.03 <= params["voxel_size"] <= 0.10
    assert params["integrate_depth_rate_hz"] <= 15.0


def test_launch_uses_aligned_depth_and_does_not_own_localization():
    source = (PACKAGE_ROOT / "launch" / "mapping.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'plugin="nvblox::NvbloxNode"' in source
    assert "aligned_depth_to_color/image_raw" in source
    assert '("camera_0/depth/camera_info", f"{camera}/color/camera_info")' in source
    assert 'DeclareLaunchArgument("start_camera", default_value="false")' in source
    assert 'DeclareLaunchArgument("global_frame", default_value="odom")' in source
    assert "rtabmap" not in source.lower()
    assert "visual_slam" not in source.lower()


def test_nav2_is_not_modified_by_first_profile():
    source = (PACKAGE_ROOT / "launch" / "mapping.launch.py").read_text(
        encoding="utf-8"
    )
    assert "nvblox_nav2" not in source
    assert "costmap" not in source.lower()


def test_mapping_monitor_checks_nonempty_outputs_and_motion():
    source = (PACKAGE_ROOT / "scripts" / "nvblox_mapping_monitor.py").read_text(
        encoding="utf-8"
    )
    assert '"mesh_nonempty"' in source
    assert '"esdf_cloud_nonempty"' in source
    assert '"esdf_slice_observed"' in source
    assert '"minimum_path_length"' in source
    assert "max_mesh_vertices" in source
    assert "max_observed_slice_cells" in source
    assert '"depth_gap_bounded"' in source
    assert '"memory_used_mb"' in source
    assert '"temperature_c"' in source
