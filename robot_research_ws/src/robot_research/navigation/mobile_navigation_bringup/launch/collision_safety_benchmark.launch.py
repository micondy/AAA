from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_navigation_bringup"))
    params_file = LaunchConfiguration("params_file")
    source_timeout = LaunchConfiguration("source_timeout")
    output_dir = LaunchConfiguration("output_dir")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"use_sim_time": "false"},
            convert_types=True,
        ),
        allow_substs=True,
    )
    monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            configured_params,
            {
                "cmd_vel_in_topic": "/safety_test/cmd_vel_in",
                "cmd_vel_out_topic": "/safety_test/cmd_vel_out",
                "state_topic": "/safety_test/state",
                "source_timeout": source_timeout,
                "front_depth.topic": "/safety_test/points",
            },
        ],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_safety_test",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["collision_monitor"],
            }
        ],
    )
    benchmark = Node(
        package="mobile_navigation_bringup",
        executable="collision_safety_benchmark.py",
        name="collision_safety_benchmark",
        output="screen",
        arguments=[
            "--source-timeout",
            source_timeout,
            "--output-dir",
            output_dir,
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config" / "nav2.yaml")
            ),
            DeclareLaunchArgument("source_timeout", default_value="0.75"),
            DeclareLaunchArgument(
                "output_dir", default_value="results/navigation/safety"
            ),
            monitor,
            lifecycle,
            TimerAction(period=2.0, actions=[benchmark]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=benchmark,
                    on_exit=[EmitEvent(event=Shutdown(reason="benchmark complete"))],
                )
            ),
        ]
    )
