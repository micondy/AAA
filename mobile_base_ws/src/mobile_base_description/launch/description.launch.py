from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_base_description"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_cad_visuals = LaunchConfiguration("use_cad_visuals")
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            str(share / "urdf" / "mobile_base.urdf.xacro"),
            " use_cad_visuals:=", use_cad_visuals,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_cad_visuals", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
