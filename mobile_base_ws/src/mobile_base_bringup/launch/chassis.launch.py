import math
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WHEELS = {
    "front_left": "fl", "front_right": "fr", "rear_left": "rl", "rear_right": "rr"
}


def _default_config(share: Path) -> Path:
    runtime_dir = Path(
        os.environ.get("MOBILE_BASE_CONFIG_DIR", Path.home() / ".config" / "mobile_base")
    ).expanduser()
    runtime = runtime_dir / "chassis.yaml"
    return runtime if runtime.is_file() else share / "config" / "chassis.yaml"


def _load_chassis(path: Path) -> dict:
    if not path.is_file():
        raise RuntimeError(f"找不到底盘配置：{path}")
    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if int(config.get("schema_version", 0)) != 1:
        raise RuntimeError("chassis.yaml schema_version 必须为 1")
    if set(config.get("wheels", {})) != set(WHEELS):
        raise RuntimeError("chassis.yaml 必须包含四个标准轮组")
    geometry = config.get("geometry", {})
    if any(float(geometry.get(name, 0)) <= 0 for name in ("wheel_radius", "wheelbase", "track_width")):
        raise RuntimeError("轮径、轴距和轮距必须大于 0")
    motor_ids, master_ids, drive_channels = set(), set(), set()
    for name in WHEELS:
        steering = config["wheels"][name]["steering"]
        drive = config["wheels"][name]["drive"]
        motor_id, master_id = int(steering["motor_id"]), int(steering["master_id"])
        pair = (int(drive["node_id"]), int(drive["channel"]))
        if not 1 <= motor_id <= 15 or not 0 <= master_id <= 0x7FF:
            raise RuntimeError(f"{name} 达妙 ID 超出范围")
        if float(steering.get("max_velocity_rad_s", 0.0)) <= 0.0:
            raise RuntimeError(f"{name} 转向速度上限必须大于 0")
        if pair[1] not in (1, 2) or not 1 <= pair[0] <= 127:
            raise RuntimeError(f"{name} ZLAC 节点或通道超出范围")
        if motor_id in motor_ids or master_id in master_ids or pair in drive_channels:
            raise RuntimeError("底盘配置存在重复的达妙 ID 或 ZLAC 节点/通道")
        motor_ids.add(motor_id)
        master_ids.add(master_id)
        drive_channels.add(pair)
    return config


def _xacro_arguments(config: dict) -> dict[str, object]:
    geometry, buses = config["geometry"], config["can"]
    result: dict[str, object] = {
        "dm_can_interface": buses["steering"]["interface"],
        "dm_can_fd": str(bool(buses["steering"].get("fd", False))).lower(),
        "zlac_can_interface": buses["drive"]["interface"],
        "wheel_radius": geometry["wheel_radius"],
        "steering_limit_rad": math.radians(config["safety"].get("steering_limit_deg", 90.0)),
        "front_x": geometry["wheelbase"] / 2.0,
        "rear_x": -geometry["wheelbase"] / 2.0,
        "left_y": geometry["track_width"] / 2.0,
        "right_y": -geometry["track_width"] / 2.0,
    }
    for name, short in WHEELS.items():
        steering = config["wheels"][name]["steering"]
        drive = config["wheels"][name]["drive"]
        result.update({
            f"{short}_motor_id": steering["motor_id"],
            f"{short}_master_id": steering["master_id"],
            f"{short}_steer_direction": steering["direction"],
            f"{short}_offset": steering["offset_rad"],
            f"{short}_steer_max_velocity": steering["max_velocity_rad_s"],
            f"{short}_node": drive["node_id"],
            f"{short}_channel": drive["channel"],
            f"{short}_drive_direction": drive["direction"],
            f"{short}_gear_ratio": drive["gear_ratio"],
            f"{short}_max_rpm": drive["max_motor_rpm"],
        })
    return result


def _controller_parameters(template_path: Path, config: dict) -> str:
    parameters = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    swerve = parameters["swerve_controller"]["ros__parameters"]
    geometry, safety = config["geometry"], config["safety"]
    swerve["wheel_radius"] = float(geometry["wheel_radius"])
    swerve["max_wheel_angular_velocity"] = float(safety.get("max_wheel_angular_velocity", 20.0))
    swerve["max_wheel_angular_acceleration"] = float(
        safety.get("max_wheel_angular_acceleration", 8.0))
    swerve["steering_limit_rad"] = math.radians(float(safety.get("steering_limit_deg", 90.0)))
    swerve["max_steering_rate_rad_s"] = min(
        float(wheel["steering"]["max_velocity_rad_s"])
        for wheel in config["wheels"].values())
    swerve["drive_alignment_tolerance_rad"] = math.radians(
        float(safety.get("drive_alignment_tolerance_deg", 5.0)))
    swerve["command_timeout"] = float(safety.get("controller_command_timeout", 0.25))
    positions = {
        "front_left": (geometry["wheelbase"] / 2.0, geometry["track_width"] / 2.0),
        "front_right": (geometry["wheelbase"] / 2.0, -geometry["track_width"] / 2.0),
        "rear_left": (-geometry["wheelbase"] / 2.0, geometry["track_width"] / 2.0),
        "rear_right": (-geometry["wheelbase"] / 2.0, -geometry["track_width"] / 2.0),
    }
    for name, (x, y) in positions.items():
        swerve[name]["x"], swerve[name]["y"] = float(x), float(y)
    temporary = tempfile.NamedTemporaryFile(
        mode="w", prefix="mobile_base_controllers_", suffix=".yaml", delete=False, encoding="utf-8"
    )
    with temporary:
        yaml.safe_dump(parameters, temporary, allow_unicode=True, sort_keys=False)
    return temporary.name


def _launch_setup(context, share: Path):
    description_share = Path(get_package_share_directory("mobile_base_description"))
    config_path = Path(LaunchConfiguration("chassis_config").perform(context)).expanduser()
    config = _load_chassis(config_path)
    controller_config = _controller_parameters(share / "config" / "controllers.yaml", config)
    xacro_command = [FindExecutable(name="xacro"), " ", str(description_share / "urdf" / "mobile_base.urdf.xacro")]
    for name, value in _xacro_arguments(config).items():
        xacro_command.extend([" ", f"{name}:={value}"])
    robot_description = ParameterValue(Command(xacro_command), value_type=str)

    control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_config], output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}], output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    swerve_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["swerve_controller", "--controller-manager", "/controller_manager"],
    )
    steering_maintenance_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["steering_maintenance_controller", "--inactive", "--controller-manager", "/controller_manager"],
    )
    drive_maintenance_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["drive_maintenance_controller", "--inactive", "--controller-manager", "/controller_manager"],
    )
    return [
        LogInfo(msg=f"使用底盘配置：{config_path}"), control_node, robot_state_publisher,
        joint_state_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[steering_maintenance_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=steering_maintenance_spawner,
                on_exit=[drive_maintenance_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=drive_maintenance_spawner,
                on_exit=[swerve_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node 已退出"))],
            )
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_base_bringup"))
    default_config = _default_config(share)
    return LaunchDescription([
        DeclareLaunchArgument(
            "chassis_config", default_value=str(default_config),
            description="统一底盘 chassis.yaml 路径",
        ),
        OpaqueFunction(function=_launch_setup, args=[share]),
    ])
