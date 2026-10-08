from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_navigation_bringup"))
    description_share = Path(get_package_share_directory("mobile_manipulator_description"))
    params = ParameterFile(
        RewrittenYaml(
            source_file=str(share / "config" / "nav2.yaml"),
            root_key="",
            param_rewrites={"use_sim_time": "true"},
            convert_types=True,
        ),
        allow_substs=True,
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(description_share / "launch" / "display.launch.py")),
        launch_arguments={
            "use_rviz": "false",
            "use_joint_state_publisher": "false",
            "use_sim_time": "true",
            "nav_camera_use_nominal_extrinsics": "true",
        }.items(),
    )
    monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            params,
            {
                "cmd_vel_in_topic": "/safety_test/cmd_vel_in",
                "cmd_vel_out_topic": "/cmd_vel",
                "state_topic": "/safety_test/state",
                "source_timeout": 0.75,
                "front_depth.topic": "/safety_test/points",
            },
        ],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_dynamic_intrusion_test",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "bond_timeout": 0.0,
            "node_names": ["collision_monitor"],
        }],
    )
    benchmark = Node(
        package="mobile_navigation_bringup",
        executable="dynamic_obstacle_intrusion_benchmark.py",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "allow_motion": ParameterValue(
                LaunchConfiguration("allow_motion"), value_type=bool
            ),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "allow_motion",
            default_value="false",
            description="Required acknowledgement that this simulation test publishes /cmd_vel",
        ),
        description,
        monitor,
        lifecycle,
        TimerAction(period=2.0, actions=[benchmark]),
        RegisterEventHandler(OnProcessExit(
            target_action=benchmark,
            on_exit=[EmitEvent(event=Shutdown(reason="benchmark complete"))],
        )),
    ])
