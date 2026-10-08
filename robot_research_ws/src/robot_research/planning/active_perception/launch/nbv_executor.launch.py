from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory("active_perception") + "/config/executor.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("execution_enabled", default_value="false"),
            DeclareLaunchArgument(
                "output_directory", default_value="/tmp/active_perception_executions"
            ),
            Node(
                package="active_perception",
                executable="nbv_executor",
                name="nbv_executor",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "execution_enabled": LaunchConfiguration("execution_enabled"),
                        "output_directory": LaunchConfiguration("output_directory"),
                    },
                ],
            ),
        ]
    )
