"""Physical Jazzy profile: four D435i cameras and two external 2-D LiDARs."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


EXPECTED_CAMERA_NAMES = {
    "nav_camera",
    "rear_camera",
    "left_camera",
    "right_camera",
}
EXPECTED_LIDAR_NAMES = {"front_lidar", "rear_lidar"}


def _vector(entry, key):
    value = entry.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError(f"{entry.get('name', 'sensor')}.{key} must contain 3 numbers")
    try:
        return [str(float(component)) for component in value]
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{entry.get('name', 'sensor')}.{key} must be numeric") from error


def _static_tf(entry, child_frame):
    xyz = _vector(entry, "xyz")
    rpy = _vector(entry, "rpy")
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{entry['name']}_mount_tf",
        arguments=[
            "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
            "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
            "--frame-id", "base_link", "--child-frame-id", child_frame,
        ],
        output="screen",
    )


def _launch_hardware(context):
    config_path = Path(
        LaunchConfiguration("sensor_config").perform(context)
    ).expanduser()
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    cameras = config.get("cameras", [])
    lidars = config.get("lidars", [])
    camera_names = {camera.get("name") for camera in cameras}
    lidar_names = {lidar.get("name") for lidar in lidars}
    if camera_names != EXPECTED_CAMERA_NAMES:
        raise RuntimeError(
            f"sensor config cameras must be exactly {sorted(EXPECTED_CAMERA_NAMES)}"
        )
    if lidar_names != EXPECTED_LIDAR_NAMES:
        raise RuntimeError(
            f"sensor config lidars must be exactly {sorted(EXPECTED_LIDAR_NAMES)}"
        )

    camera_launch = Path(get_package_share_directory("d435i_bringup")) / "launch" / "camera.launch.py"
    actions = []
    for camera in cameras:
        serial = str(camera.get("serial_no", "")).strip()
        if not serial or serial.startswith("CHANGE_ME"):
            raise RuntimeError(
                f"Set the measured serial_no for {camera['name']} in {config_path}"
            )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(camera_launch)),
                launch_arguments={
                    "camera_namespace": "sensors",
                    "camera_name": camera["name"],
                    "serial_no": serial,
                    "stage": "full",
                    "reset_on_start": LaunchConfiguration("reset_cameras"),
                    "use_sim_time": "false",
                    "use_rviz": "false",
                }.items(),
            )
        )
        if camera.get("publish_tf", False):
            actions.append(_static_tf(camera, f"{camera['name']}_link"))

    for lidar in lidars:
        topic = str(lidar.get("topic", ""))
        frame_id = str(lidar.get("frame_id", ""))
        if not topic.startswith("/") or not frame_id:
            raise RuntimeError(f"Invalid topic/frame_id for {lidar['name']}")
        if lidar.get("publish_tf", False):
            actions.append(_static_tf(lidar, frame_id))

    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_navigation_bringup"))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "navigation.launch.py")),
        launch_arguments={
            "start_camera": "false",
            "camera_namespace": "sensors",
            "camera_name": "nav_camera",
            "odom_source": "external",
            "odom_topic": LaunchConfiguration("odom_topic"),
            "database_path": LaunchConfiguration("database_path"),
            "initial_pose": LaunchConfiguration("initial_pose"),
            "rtabmap_mode": LaunchConfiguration("rtabmap_mode"),
            "fresh_map": LaunchConfiguration("fresh_map"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "costmap_overlay_file": LaunchConfiguration("costmap_overlay_file"),
            "cmd_vel_out_topic": LaunchConfiguration("cmd_vel_out_topic"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "sensor_config",
            default_value=str(share / "config" / "hardware_sensors.yaml"),
        ),
        DeclareLaunchArgument(
            "costmap_overlay_file",
            default_value=str(share / "config" / "costmap_multi_sensor_overlay.yaml"),
        ),
        DeclareLaunchArgument("reset_cameras", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/swerve_controller/odom"),
        DeclareLaunchArgument("cmd_vel_out_topic", default_value="/swerve_controller/cmd_vel"),
        DeclareLaunchArgument("initial_pose", default_value=""),
        DeclareLaunchArgument("rtabmap_mode", default_value="localization"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument("database_path"),
        OpaqueFunction(function=_launch_hardware),
        navigation,
    ])
