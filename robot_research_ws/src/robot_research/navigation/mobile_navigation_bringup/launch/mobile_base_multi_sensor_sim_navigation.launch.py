"""Isaac Sim navigation profile for four RGB-D cameras and two RTX LiDARs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navigation_share = Path(get_package_share_directory("mobile_navigation_bringup"))
    description_share = Path(get_package_share_directory("mobile_manipulator_description"))

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(description_share / "launch" / "display.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_joint_state_publisher": "false",
            "use_sim_time": "true",
            "nav_camera_use_nominal_extrinsics": "true",
            "surround_sensors_enabled": "true",
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "navigation.launch.py")
        ),
        launch_arguments={
            "start_camera": "false",
            "use_sim_time": "true",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "database_path": LaunchConfiguration("database_path"),
            "rtabmap_mode": LaunchConfiguration("rtabmap_mode"),
            "fresh_map": LaunchConfiguration("fresh_map"),
            "multi_sensor_mapping": "true",
            "initial_pose": LaunchConfiguration("initial_pose"),
            "odom_source": "external",
            "odom_topic": "/swerve_controller/odom",
            "cmd_vel_out_topic": "/cmd_vel_nav_safe",
            "cmd_vel_smoothed_topic": "/cmd_vel_smoothed",
            "collision_source_timeout": "2.0",
            "costmap_overlay_file": str(
                navigation_share / "config" / "costmap_multi_sensor_overlay.yaml"
            ),
        }.items(),
    )
    command_mux = Node(
        package="mobile_navigation_bringup",
        executable="cmd_vel_priority_mux.py",
        name="cmd_vel_priority_mux",
        output="screen",
    )
    web_console = Node(
        package="mobile_base_web_console",
        executable="web_console",
        name="mobile_base_web_console",
        remappings=[
            ("/swerve_controller/cmd_vel", "/cmd_vel_web"),
            ("/map", "/rtabmap/map"),
        ],
        additional_env={"MOBILE_BASE_SIMULATION_DIRECT_CONTROL": "true"},
        condition=IfCondition(LaunchConfiguration("start_web_console")),
        output="screen",
    )
    camera_viewer = Node(
        package="mobile_navigation_bringup",
        executable="four_camera_viewer.py",
        name="four_camera_viewer",
        parameters=[{
            "host": LaunchConfiguration("camera_viewer_host"),
            "port": ParameterValue(LaunchConfiguration("camera_viewer_port"), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration("start_camera_viewer")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("start_web_console", default_value="true"),
        DeclareLaunchArgument("start_camera_viewer", default_value="true"),
        DeclareLaunchArgument("camera_viewer_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("camera_viewer_port", default_value="8090"),
        DeclareLaunchArgument("database_path"),
        DeclareLaunchArgument("rtabmap_mode", default_value="mapping"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument("initial_pose", default_value="0 0 0 0 0 0"),
        description,
        navigation,
        command_mux,
        web_console,
        camera_viewer,
    ])
