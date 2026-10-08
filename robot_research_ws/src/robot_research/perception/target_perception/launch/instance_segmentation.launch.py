from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("target_perception"))
    default_params = share / "config" / "instance_segmentation.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("output_frame", default_value="base_link"),
            DeclareLaunchArgument("weights_path", default_value=""),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("primary_class_id", default_value=""),
            Node(
                package="target_perception",
                executable="instance_segmentation",
                name="instance_segmentation",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "output_frame": LaunchConfiguration("output_frame"),
                        "weights_path": LaunchConfiguration("weights_path"),
                        "device": LaunchConfiguration("device"),
                        "primary_class_id": LaunchConfiguration(
                            "primary_class_id"
                        ),
                    },
                ],
            ),
        ]
    )
