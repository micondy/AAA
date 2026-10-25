"""Isaac rack grid with online mapping, Nav2 and the web control console."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("mobile_manipulator_bringup"))
    navigation_share = Path(get_package_share_directory("mobile_navigation_bringup"))
    default_database = str(Path.home() / ".ros" / "rack_grid4x4_online.db")

    racks = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "sim_rack.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_rack_stack")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "test_seconds": LaunchConfiguration("test_seconds"),
            "enable_nvblox": LaunchConfiguration("enable_nvblox"),
            "rack_layout": "grid4x4",
            "rack_visual_seed": LaunchConfiguration("rack_visual_seed"),
            "anonymous_instances": LaunchConfiguration("anonymous_instances"),
            "use_yolo": LaunchConfiguration("use_yolo"),
            "yolo_weights_path": LaunchConfiguration("yolo_weights_path"),
            "yolo_python_executable": LaunchConfiguration("yolo_python_executable"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(navigation_share / "launch" / "navigation.launch.py")),
        launch_arguments={
            "start_camera": "false",
            "use_rviz": "false",
            "use_sim_time": "true",
            # nvblox's nested launch also declares "params_file". Pass the
            # Nav2 file explicitly so the two includes cannot share it.
            "params_file": str(navigation_share / "config" / "nav2.yaml"),
            "database_path": LaunchConfiguration("database_path"),
            "rtabmap_mode": "mapping",
            "fresh_map": "false",
            "odom_source": "external",
            "odom_topic": "/swerve_controller/odom",
            "navigation_startup_delay": "15.0",
            "xy_goal_tolerance": "0.08",
            "yaw_goal_tolerance": "0.15",
            "required_movement_radius": "0.08",
            "cmd_vel_smoothed_topic": "/cmd_vel_nav_smoothed",
            "cmd_vel_in_topic": "/cmd_vel_muxed",
            "cmd_vel_out_topic": "/cmd_vel",
            "collision_source_timeout": "2.0",
            "costmap_overlay_file": str(
                navigation_share / "config" / "costmap_rack_grid_overlay.yaml"
            ),
        }.items(),
    )
    web = Node(
        package="mobile_base_web_console",
        executable="web_console",
        name="mobile_base_web_console",
        output="screen",
        remappings=[
            ("/swerve_controller/cmd_vel", "/cmd_vel_web"),
            ("/map", "/rtabmap/map"),
        ],
        additional_env={"MOBILE_BASE_SIMULATION_DIRECT_CONTROL": "true"},
        condition=IfCondition(LaunchConfiguration("start_web_console")),
    )
    mux = Node(
        package="mobile_navigation_bringup",
        executable="cmd_vel_priority_mux.py",
        name="cmd_vel_priority_mux",
        output="screen",
        parameters=[{
            "nav_input_topic": "/cmd_vel_nav_smoothed",
            "manual_input_topic": "/cmd_vel_web",
            "output_topic": "/cmd_vel_muxed",
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            str(Path(get_package_share_directory("nav2_bringup")) / "rviz" / "nav2_default_view.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rack_stack",
            default_value="true",
            description="Set false to attach Nav2 and web to an already running sim_rack.launch.py",
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("test_seconds", default_value="0"),
        DeclareLaunchArgument("enable_nvblox", default_value="false"),
        DeclareLaunchArgument("anonymous_instances", default_value="false"),
        DeclareLaunchArgument("rack_visual_seed", default_value="0"),
        DeclareLaunchArgument("use_yolo", default_value="false"),
        DeclareLaunchArgument("yolo_weights_path", default_value=""),
        DeclareLaunchArgument("yolo_python_executable", default_value="python3"),
        DeclareLaunchArgument("start_web_console", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("database_path", default_value=default_database),
        racks,
        navigation,
        mux,
        web,
        rviz,
    ])
