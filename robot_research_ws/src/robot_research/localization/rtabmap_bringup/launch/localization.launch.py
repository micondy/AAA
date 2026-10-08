from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("rtabmap_bringup"))
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "online.launch.py")),
        launch_arguments={
            "start_camera": LaunchConfiguration("start_camera"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "camera_name": LaunchConfiguration("camera_name"),
            "serial_no": LaunchConfiguration("serial_no"),
            "camera_stage": LaunchConfiguration("camera_stage"),
            "base_frame": LaunchConfiguration("base_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "map_frame": LaunchConfiguration("map_frame"),
            "initial_pose": LaunchConfiguration("initial_pose"),
            "database_path": LaunchConfiguration("database_path"),
            "fresh_map": "false",
            "mode": "localization",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("database_path", description="Existing RTAB-Map database"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("camera_stage", default_value="aligned"),
        DeclareLaunchArgument("base_frame", default_value="camera_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("odom_source", default_value="rgbd"),
        DeclareLaunchArgument("odom_topic", default_value="/swerve_controller/odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("initial_pose", default_value=""),
        localization,
    ])
