import importlib.util
import subprocess
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
DESCRIPTION_ROOT = ROOT.parent / "mobile_base_description"
SPEC = importlib.util.spec_from_file_location("chassis_launch", ROOT / "launch" / "chassis.launch.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def config():
    return yaml.safe_load((ROOT / "config" / "chassis.yaml").read_text(encoding="utf-8"))


def test_chassis_config_maps_to_xacro_and_controller(tmp_path):
    chassis = config()
    chassis["geometry"].update(wheel_radius=0.12, wheelbase=0.8, track_width=0.6)
    chassis["safety"].update(
        max_wheel_angular_acceleration=7.5,
    )
    chassis["wheels"]["front_left"]["steering"]["offset_rad"] = 1.234
    chassis["wheels"]["front_left"]["steering"]["max_velocity_rad_s"] = 0.08
    arguments = MODULE._xacro_arguments(chassis)
    assert arguments["wheel_radius"] == 0.12
    assert arguments["front_x"] == 0.4 and arguments["left_y"] == 0.3
    assert arguments["fl_motor_id"] == 2 and arguments["fl_master_id"] == 0x12
    assert arguments["fl_offset"] == 1.234

    generated = Path(MODULE._controller_parameters(ROOT / "config" / "controllers.yaml", chassis))
    try:
        controllers = yaml.safe_load(generated.read_text(encoding="utf-8"))
        params = controllers["swerve_controller"]["ros__parameters"]
        assert params["wheel_radius"] == 0.12
        assert params["max_wheel_angular_acceleration"] == 7.5
        assert params["max_steering_rate_rad_s"] == 0.08
        assert params["front_left"]["x"] == 0.4
        assert params["front_right"]["y"] == -0.3
    finally:
        generated.unlink()


def test_xacro_accepts_all_generated_arguments():
    chassis = config()
    arguments = MODULE._xacro_arguments(chassis)
    command = ["xacro", str(DESCRIPTION_ROOT / "urdf" / "mobile_base.urdf.xacro")]
    command.extend(f"{name}:={value}" for name, value in arguments.items())
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    assert '<param name="dm_can_interface">can1</param>' in result.stdout
    assert '<param name="zlac_can_interface">can0</param>' in result.stdout
    assert '<param name="motor_id">2</param>' in result.stdout
    assert '<param name="master_id">18</param>' in result.stdout
    assert '<param name="node_id">5</param>' in result.stdout
    assert '<param name="node_id">6</param>' in result.stdout


def test_launch_rejects_duplicate_motor_id(tmp_path):
    chassis = config()
    chassis["wheels"]["rear_right"]["steering"]["motor_id"] = 1
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(chassis), encoding="utf-8")
    try:
        MODULE._load_chassis(path)
        assert False, "重复 ID 必须被拒绝"
    except RuntimeError as error:
        assert "重复" in str(error)
