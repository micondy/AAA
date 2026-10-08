from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("rtabmap_bringup"))
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "online.launch.py")),
        launch_arguments={
            "start_camera": "false",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": "true",
            "base_frame": LaunchConfiguration("base_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "map_frame": LaunchConfiguration("map_frame"),
            "database_path": LaunchConfiguration("database_path"),
            "fresh_map": LaunchConfiguration("fresh_map"),
            "mode": "mapping",
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "camera_name": LaunchConfiguration("camera_name"),
        }.items(),
    )
    playback = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", LaunchConfiguration("bag"),
                "--clock", "--rate", LaunchConfiguration("rate"),
            ],
            output="screen",
            sigterm_timeout="10",
            sigkill_timeout="5",
        )],
    )
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="Existing D435i rosbag directory"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("base_frame", default_value="camera_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/rtabmap_offline.db"
        ),
        mapping,
        playback,
    ])
