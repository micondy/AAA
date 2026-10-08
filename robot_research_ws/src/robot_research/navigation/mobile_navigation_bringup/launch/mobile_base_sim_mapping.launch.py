from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = Path(get_package_share_directory("mobile_navigation_bringup"))
    description_share = Path(get_package_share_directory("mobile_manipulator_description"))
    rtabmap_share = Path(get_package_share_directory("rtabmap_bringup"))
    nvblox_share = Path(get_package_share_directory("nvblox_bringup"))

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(description_share / "launch" / "display.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_joint_state_publisher": "false",
            "use_sim_time": "true",
            "nav_camera_use_nominal_extrinsics": "true",
        }.items(),
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "mapping.launch.py")
        ),
        launch_arguments={
            "start_camera": "false",
            "use_sim_time": "true",
            "use_rviz": "false",
            "fresh_map": LaunchConfiguration("fresh_map"),
            "database_path": LaunchConfiguration("database_path"),
            "odom_source": "external",
            "odom_topic": "/swerve_controller/odom",
        }.items(),
    )
    def launch_rviz(context):
        # Evaluate this before nested launch files set their own use_rviz=false.
        enabled = LaunchConfiguration("use_rviz").perform(context).lower()
        if enabled not in ("true", "1", "yes"):
            return []
        return [Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rtabmap_share / "rviz" / "rtabmap.rviz")],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )]
    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nvblox_share / "launch" / "mapping.launch.py")
        ),
        launch_arguments={
            "start_camera": "false",
            "use_rviz": "false",
            "use_sim_time": "true",
            "global_frame": "odom",
            "base_frame": "base_link",
            "input_qos": "DEFAULT",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_nvblox")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("enable_nvblox", default_value="false"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/isaac_mobile_base_map.db",
            description="RTAB-Map database for the custom mobile-base simulation",
        ),
        OpaqueFunction(function=launch_rviz),
        description,
        mapping,
        nvblox,
    ])
