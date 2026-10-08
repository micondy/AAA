from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


MODULES = ("front_left", "front_right", "rear_left", "rear_right")


def _render(*arguments: str) -> ET.Element:
    xacro_file = (
        Path(get_package_share_directory("mobile_base_description"))
        / "urdf"
        / "mobile_base.urdf.xacro"
    )
    result = subprocess.run(
        ["xacro", str(xacro_file), "use_mock_hardware:=true", *arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def test_cad_model_uses_clean_swerve_axes_and_simple_collisions():
    robot = _render("use_cad_visuals:=true")
    base = robot.find("link[@name='base_link']")
    assert base is not None
    assert base.find("visual/geometry/mesh") is not None
    assert base.find("collision/geometry/box") is not None
    assert base.find("inertial") is not None

    expected_xy = {
        "front_left": "0.2095 0.2250 0",
        "front_right": "0.2095 -0.2250 0",
        "rear_left": "-0.2095 0.2250 0",
        "rear_right": "-0.2095 -0.2250 0",
    }
    for name in MODULES:
        steer = robot.find(f"joint[@name='{name}_steer_joint']")
        drive = robot.find(f"joint[@name='{name}_drive_joint']")
        wheel = robot.find(f"link[@name='{name}_wheel_link']")
        assert steer.find("origin").attrib["xyz"] == expected_xy[name]
        assert steer.find("axis").attrib["xyz"] == "0 0 1"
        assert drive.find("axis").attrib["xyz"] == "0 1 0"
        drive_origin = tuple(
            float(value) for value in drive.find("origin").attrib["xyz"].split()
        )
        assert drive_origin in {(0.0, 0.043, 0.0), (0.0, -0.043, 0.0)}
        assert wheel.find("visual/geometry/mesh") is not None
        assert wheel.find("collision/geometry/cylinder") is not None
        assert wheel.find("inertial") is not None


def test_primitive_preview_does_not_require_cad_meshes():
    robot = _render("use_cad_visuals:=false")
    assert not robot.findall(".//visual/geometry/mesh")
    assert robot.find("link[@name='base_link']/visual/geometry/box") is not None
    for name in MODULES:
        assert (
            robot.find(f"link[@name='{name}_wheel_link']/visual/geometry/cylinder")
            is not None
        )
