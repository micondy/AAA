from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_manipulator_description"))
    xacro_file = share / "urdf" / "mobile_manipulator.urdf.xacro"
    rviz_file = share / "rviz" / "mobile_manipulator.rviz"

    robot_description = {
        "robot_description": ParameterValue(Command([
            FindExecutable(name="xacro"),
            " ", str(xacro_file),
            " use_mock_hardware:=true",
            " openarm_ros2_control:=false",
            " openarm_mount_xyz:='", LaunchConfiguration("openarm_mount_xyz"), "'",
            " openarm_mount_rpy:='", LaunchConfiguration("openarm_mount_rpy"), "'",
            " nav_camera_enabled:=", LaunchConfiguration("nav_camera_enabled"),
            " nav_camera_mount_xyz:='", LaunchConfiguration("nav_camera_mount_xyz"), "'",
            " nav_camera_mount_rpy:='", LaunchConfiguration("nav_camera_mount_rpy"), "'",
            " nav_camera_use_nominal_extrinsics:=", LaunchConfiguration("nav_camera_use_nominal_extrinsics"),
        ]), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("openarm_mount_xyz", default_value="0 0 0.18"),
        DeclareLaunchArgument("openarm_mount_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("nav_camera_enabled", default_value="true"),
        DeclareLaunchArgument("nav_camera_mount_xyz", default_value="0.10 0 1.70"),
        DeclareLaunchArgument("nav_camera_mount_rpy", default_value="0 0.20944 0"),
        DeclareLaunchArgument("nav_camera_use_nominal_extrinsics", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                robot_description,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
            parameters=[
                robot_description,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", str(rviz_file)],
            parameters=[
                robot_description,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            output="screen",
        ),
    ])
