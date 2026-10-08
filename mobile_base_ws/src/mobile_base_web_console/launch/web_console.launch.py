from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mobile_base_web_console",
                executable="web_console",
                name="mobile_base_web_console",
                output="screen",
            )
        ]
    )
