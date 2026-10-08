"""独立关节姿态预览：只有模型、滑条和 RViz，不加载硬件控制器。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_base_description"))
    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        str(share / "urdf" / "mobile_base.urdf.xacro"),
        " use_cad_visuals:=", LaunchConfiguration("use_cad_visuals"),
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("use_cad_visuals", default_value="true"),
        # 独立通信域，避免模拟 joint_states 混入实车反馈。
        SetEnvironmentVariable("ROS_DOMAIN_ID", "87"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             name="joint_preview_sliders", output="screen"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(share / "rviz" / "description.rviz")],
             name="joint_preview_rviz", output="screen"),
    ])
