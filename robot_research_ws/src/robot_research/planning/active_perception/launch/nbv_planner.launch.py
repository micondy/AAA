from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory("active_perception") + "/config/nbv.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            Node(
                package="active_perception",
                executable="nbv_planner",
                name="nbv_planner",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "map_topic": LaunchConfiguration("map_topic"),
                    },
                ],
            ),
        ]
    )
