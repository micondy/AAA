from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("target_perception"))
    default_params = share / "config" / "color_depth.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("output_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "color_topic",
                default_value="/sensors/nav_camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/sensors/nav_camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/sensors/nav_camera/color/camera_info",
            ),
            Node(
                package="target_perception",
                executable="color_depth_detector",
                name="color_depth_detector",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "output_frame": LaunchConfiguration("output_frame"),
                        "color_topic": LaunchConfiguration("color_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    },
                ],
            ),
        ]
    )
