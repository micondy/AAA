"""Fixed-base MoveIt mock plus a configurable generic task environment."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("mobile_manipulator_bringup"))
    moveit_share = Path(
        get_package_share_directory("mobile_manipulator_moveit_config")
    )
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_share / "launch" / "demo.launch.py")),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "fixed_base": "true",
            "start_base_controller": "false",
        }.items(),
    )
    scene = Node(
        package="mobile_manipulator_bringup",
        executable="apply_task_scene",
        name="task_scene_applicator",
        output="screen",
        parameters=[
            {
                "scene_file": LaunchConfiguration("scene_file"),
                "operation": "add",
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "scene_file",
                default_value=str(bringup_share / "config" / "generic_task_scene.yaml"),
            ),
            moveit_demo,
            TimerAction(period=4.0, actions=[scene]),
        ]
    )
