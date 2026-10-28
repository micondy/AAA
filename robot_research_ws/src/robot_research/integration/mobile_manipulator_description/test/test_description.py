from pathlib import Path
import ast
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def _render_description(*xacro_args):
    package_share = Path(get_package_share_directory("mobile_manipulator_description"))
    xacro_file = package_share / "urdf" / "mobile_manipulator.urdf.xacro"
    result = subprocess.run(
        [
            "xacro",
            str(xacro_file),
            "use_mock_hardware:=true",
            "openarm_ros2_control:=false",
            *xacro_args,
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def test_integrated_description_is_one_valid_tree():
    package_share = Path(get_package_share_directory("mobile_manipulator_description"))
    official_source = Path(__file__).resolve().parents[4] / "vendor" / "openarm_description"
    resolved_openarm = Path(
        get_package_share_directory("openarm_description")
    ) / "urdf" / "robot" / "v10.urdf.xacro"
    assert resolved_openarm.resolve().is_relative_to(official_source.resolve())

    root = _render_description()
    links = {node.attrib["name"] for node in root.findall("link")}
    joints = {node.attrib["name"] for node in root.findall("joint")}

    assert len(links) == len(root.findall("link"))
    assert len(joints) == len(root.findall("joint"))
    assert "world" not in links
    assert "base_link" in links
    assert "openarm_mount_link" in links
    assert "openarm_body_link0" in links
    assert "openarm_left_link7" in links
    assert "openarm_right_link7" in links
    assert "openarm_left_hand_tcp" in links
    assert "openarm_right_hand_tcp" in links
    assert "nav_camera_bottom_screw_frame" in links
    assert "nav_camera_link" in links
    assert "base_to_openarm_mount" in joints
    assert "openarm_mount_to_body" in joints
    assert "front_left_steer_joint" in joints
    assert "front_left_drive_joint" in joints
    assert "openarm_left_joint7" in joints
    assert "openarm_right_joint7" in joints
    assert "nav_camera_joint" in joints
    assert "nav_camera_link_joint" in joints

    camera_mount = root.find("joint[@name='nav_camera_joint']")
    assert camera_mount is not None
    assert camera_mount.find("parent").attrib["link"] == "base_link"
    assert camera_mount.find("origin").attrib["xyz"] == "0.10 0 1.70"
    assert camera_mount.find("origin").attrib["rpy"] == "0 0.20944 0"
    camera_link = root.find("link[@name='nav_camera_link']")
    assert camera_link is not None
    assert camera_link.find("visual") is not None
    assert camera_link.find("collision") is not None
    assert camera_link.find("inertial") is not None

    child_links = {
        joint.find("child").attrib["link"]
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    assert links - child_links == {"base_link"}


def test_camera_can_be_disabled_without_changing_the_robot_root():
    root = _render_description("nav_camera_enabled:=false")
    links = {node.attrib["name"] for node in root.findall("link")}
    child_links = {
        joint.find("child").attrib["link"]
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    assert not any(name.startswith("nav_camera") for name in links)
    assert links - child_links == {"base_link"}


def test_simulation_can_request_nominal_realsense_frames():
    root = _render_description("nav_camera_use_nominal_extrinsics:=true")
    links = {node.attrib["name"] for node in root.findall("link")}
    assert "nav_camera_color_optical_frame" in links
    assert "nav_camera_depth_optical_frame" in links
    assert "nav_camera_accel_optical_frame" in links
    assert "nav_camera_gyro_optical_frame" in links


def test_surround_sensor_frames_are_opt_in_and_connected_to_base():
    default = _render_description("nav_camera_use_nominal_extrinsics:=true")
    default_links = {node.attrib["name"] for node in default.findall("link")}
    assert "rear_camera_link" not in default_links

    root = _render_description(
        "nav_camera_use_nominal_extrinsics:=true",
        "surround_sensors_enabled:=true",
    )
    links = {node.attrib["name"] for node in root.findall("link")}
    for name in ("rear_camera", "left_camera", "right_camera"):
        assert f"{name}_link" in links
        assert f"{name}_color_optical_frame" in links
    for name in ("front_lidar", "rear_lidar"):
        assert f"{name}_link" in links
        joint = root.find(f"joint[@name='base_to_{name}']")
        assert joint.find("parent").attrib["link"] == "base_link"


def test_display_launch_forces_robot_description_to_string():
    launch_file = (
        Path(get_package_share_directory("mobile_manipulator_description"))
        / "launch"
        / "display.launch.py"
    )
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)
    assert "ParameterValue(Command([" in source
    assert "value_type=str" in source
