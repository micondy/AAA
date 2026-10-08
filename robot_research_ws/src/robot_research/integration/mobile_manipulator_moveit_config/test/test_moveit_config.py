from pathlib import Path
import ast
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


ARM_JOINTS = {
    *(f"openarm_left_joint{i}" for i in range(1, 8)),
    *(f"openarm_right_joint{i}" for i in range(1, 8)),
}


def _render_urdf():
    path = (
        Path(get_package_share_directory("mobile_manipulator_description"))
        / "urdf"
        / "mobile_manipulator.urdf.xacro"
    )
    return ET.fromstring(
        xacro.process_file(
            str(path),
            mappings={
                "use_mock_hardware": "true",
                "openarm_ros2_control": "true",
                "openarm_use_fake_hardware": "true",
                "hand": "true",
                "nav_camera_enabled": "true",
                "nav_camera_use_nominal_extrinsics": "false",
            },
        ).toxml()
    )


def _render_srdf():
    path = (
        Path(get_package_share_directory("mobile_manipulator_moveit_config"))
        / "config"
        / "mobile_manipulator.srdf.xacro"
    )
    return ET.fromstring(xacro.process_file(str(path)).toxml())


def test_official_openarm_config_is_extended_by_combined_robot():
    urdf = _render_urdf()
    srdf = _render_srdf()
    urdf_joints = {node.attrib["name"] for node in urdf.findall("joint")}
    srdf_group_joints = {
        node.attrib["name"]
        for group in srdf.findall("group")
        for node in group.findall("joint")
    }

    assert urdf.attrib["name"] == "mobile_manipulator"
    assert srdf.attrib["name"] == "mobile_manipulator"
    assert ARM_JOINTS <= urdf_joints
    assert ARM_JOINTS <= srdf_group_joints
    assert srdf.find("virtual_joint[@child_link='base_link']") is not None
    assert "nav_camera_link" in {node.attrib["name"] for node in urdf.findall("link")}

    urdf_links = {node.attrib["name"] for node in urdf.findall("link")}
    for disabled in srdf.findall("disable_collisions"):
        assert disabled.attrib["link1"] in urdf_links
        assert disabled.attrib["link2"] in urdf_links

    chassis = urdf.find("link[@name='base_link']")
    assert chassis is not None
    assert chassis.find("collision") is not None
    disabled_pairs = {
        frozenset((node.attrib["link1"], node.attrib["link2"]))
        for node in srdf.findall("disable_collisions")
    }
    assert frozenset(("base_link", "openarm_mount_link")) in disabled_pairs


def test_official_moveit_and_ros2_control_controller_names_match():
    official_moveit = (
        Path(get_package_share_directory("openarm_bimanual_moveit_config"))
        / "config"
        / "moveit_controllers.yaml"
    )
    official_bringup = (
        Path(get_package_share_directory("openarm_bringup"))
        / "config"
        / "v10_controllers"
        / "openarm_v10_bimanual_controllers.yaml"
    )
    with official_moveit.open(encoding="utf-8") as stream:
        moveit = yaml.safe_load(stream)
    with official_bringup.open(encoding="utf-8") as stream:
        control = yaml.safe_load(stream)

    moveit_names = set(moveit["moveit_simple_controller_manager"]["controller_names"])
    declared = set(control["controller_manager"]["ros__parameters"])
    assert moveit_names <= declared


def test_mock_description_does_not_load_real_openarm_hardware():
    urdf_xml = ET.tostring(_render_urdf(), encoding="unicode")
    assert "mock_components/GenericSystem" in urdf_xml
    assert "openarm_hardware/OpenArm_v10HW" not in urdf_xml


def test_mock_launch_publishes_world_to_base_tf():
    launch_file = (
        Path(get_package_share_directory("mobile_manipulator_moveit_config"))
        / "launch"
        / "demo.launch.py"
    )
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)
    assert 'executable="static_transform_publisher"' in source
    assert '"--frame-id", "world"' in source
    assert '"--child-frame-id", "base_link"' in source
    assert '"--frame-id", "world"' in source
    assert '"--child-frame-id", "odom"' in source
    assert 'DeclareLaunchArgument(\n                "fixed_base"' in source
    assert 'DeclareLaunchArgument(\n                "start_base_controller"' in source
    assert '"swerve_controller", "-c", "/controller_manager"' in source


def test_all_openarm_joints_have_positive_acceleration_limits():
    limits_file = (
        Path(get_package_share_directory("mobile_manipulator_moveit_config"))
        / "config"
        / "joint_limits.yaml"
    )
    with limits_file.open(encoding="utf-8") as stream:
        limits = yaml.safe_load(stream)["joint_limits"]

    expected = ARM_JOINTS | {
        "openarm_left_finger_joint1",
        "openarm_right_finger_joint1",
    }
    assert expected <= set(limits)
    for joint_name in expected:
        joint_limit = limits[joint_name]
        assert joint_limit["has_acceleration_limits"] is True
        assert joint_limit["max_acceleration"] > 0.0
