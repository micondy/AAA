from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("target_perception"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(share / "config" / "instance_tracking.yaml"),
            ),
            Node(
                package="target_perception",
                executable="instance_tracking",
                name="instance_tracking",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
