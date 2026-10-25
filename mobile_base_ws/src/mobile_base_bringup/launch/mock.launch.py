from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("mobile_base_description"))
    share = Path(get_package_share_directory("mobile_base_bringup"))
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(description_share / "urdf" / "mobile_base.urdf.xacro"),
                " use_mock_hardware:=true",
            ]
        ),
        value_type=str,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            str(share / "config" / "controllers.yaml"),
        ],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    swerve_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_controller", "--controller-manager", "/controller_manager"],
    )
    steering_maintenance_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["steering_maintenance_controller", "--inactive", "--controller-manager", "/controller_manager"],
    )
    drive_maintenance_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["drive_maintenance_controller", "--inactive", "--controller-manager", "/controller_manager"],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(share / "rviz" / "mock.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Start RViz2"
            ),
            DeclareLaunchArgument(
                "start_web_console", default_value="false", description="Start web console"
            ),
            Node(
                package="mobile_base_web_console", executable="web_console",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_web_console")),
            ),
            control_node,
            robot_state_publisher,
            joint_state_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[steering_maintenance_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=steering_maintenance_spawner,
                    on_exit=[drive_maintenance_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=drive_maintenance_spawner,
                    on_exit=[swerve_spawner],
                )
            ),
            rviz,
        ]
    )
