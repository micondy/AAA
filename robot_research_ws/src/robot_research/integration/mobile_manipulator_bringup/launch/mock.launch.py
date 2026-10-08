"""Single safe entry point for the complete mock mobile manipulator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_launch = Path(
        get_package_share_directory("mobile_manipulator_moveit_config")
    ) / "launch" / "demo.launch.py"

    complete_mock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_launch)),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "fixed_base": "false",
            "start_base_controller": "true",
        }.items(),
    )
    base_io = Node(
        package="mobile_manipulator_bringup",
        executable="base_io_adapter",
        output="screen",
        parameters=[
            {
                "publish_odom_tf": True,
                # GenericSystem feedback is deterministic mock truth. Isaac Sim
                # will own this topic in the later sim profile.
                "publish_ground_truth": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            complete_mock,
            base_io,
        ]
    )
