import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_base_bringup"))
    runtime_dir = Path(
        os.environ.get("MOBILE_BASE_CONFIG_DIR", Path.home() / ".config" / "mobile_base")
    ).expanduser()
    runtime_config = runtime_dir / "chassis.yaml"
    default_config = (
        runtime_config
        if runtime_config.is_file()
        else share / "config" / "chassis.yaml"
    )

    chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "chassis.launch.py")),
        launch_arguments={
            "chassis_config": LaunchConfiguration("chassis_config"),
        }.items(),
    )
    web_console = Node(
        package="mobile_base_web_console",
        executable="web_console",
        name="mobile_base_web_console",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("start_web_console")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "chassis_config",
                default_value=str(default_config),
                description="机器人端唯一底盘配置文件",
            ),
            DeclareLaunchArgument(
                "start_web_console",
                default_value="true",
                description="是否启动机器人端网页控制台后端",
            ),
            chassis,
            web_console,
        ]
    )
