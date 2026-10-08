from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_navigation_bringup"))
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "mapping.launch.py")),
        launch_arguments={
            "start_camera": "false",
            "use_sim_time": "true",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "fresh_map": LaunchConfiguration("fresh_map"),
            "database_path": LaunchConfiguration("database_path"),
        }.items(),
    )
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="carter_nav_camera_static_tf",
        arguments=[
            "--x", "0.100", "--y", "0.075", "--z", "0.346",
            "--roll", "-1.5707963268", "--pitch", "0.0", "--yaw", "-1.5707963268",
            "--frame-id", "base_link",
            "--child-frame-id", "front_stereo_camera_left_rgb",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/isaac_carter_map.db",
            description="RTAB-Map database used by the Carter simulation",
        ),
        camera_tf,
        mapping,
    ])
