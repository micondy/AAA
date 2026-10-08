from pathlib import Path
import ast


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_real_mobile_mapping_launch_has_one_consistent_camera_and_tf_tree():
    source = (PACKAGE_ROOT / "launch" / "real_mobile_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    ast.parse(source)
    assert '"camera_namespace": "sensors"' in source
    assert '"camera_name": "nav_camera"' in source
    assert '"base_frame": "base_link"' in source
    assert '"global_frame": "odom"' in source
    assert "nav_camera_use_nominal_extrinsics:=false" in source
    assert '"start_camera": "false"' in source
    assert '"/sensors/nav_camera/color/image_raw"' in source
    assert '"/sensors/nav_camera/aligned_depth_to_color/image_raw"' in source


def test_real_mobile_mapping_exposes_tf_owner_switch_for_external_odometry():
    source = (PACKAGE_ROOT / "launch" / "real_mobile_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"odom_source",' in source
    assert '"publish_odom_tf",' in source
    assert '"odom_topic", default_value="/swerve_controller/odom"' in source


def test_recorder_does_not_starve_tf_while_waiting_for_rgbd_odometry():
    source = (
        PACKAGE_ROOT / "gaussian_splatting_bringup" / "dataset_recorder.py"
    ).read_text(encoding="utf-8")
    ast.parse(source)
    assert "TransformListener(self.tf_buffer, self)" in source
    assert "MultiThreadedExecutor(num_threads=2)" in source
    assert 'declare_parameter("tf_timeout_s", 0.5)' in source
    assert 'declare_parameter("pose_delay_s", 0.25)' in source
    assert "self.pending_frames" in source
    assert source.index("executor.shutdown()") < source.index("node.close()")
