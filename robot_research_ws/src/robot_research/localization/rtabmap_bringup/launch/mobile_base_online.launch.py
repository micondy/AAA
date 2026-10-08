"""Real mobile-base online RTAB-Map using wheel odometry and a live D435i."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rtabmap_bringup"))

    camera_mount = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="nav_camera_mount_tf",
        arguments=[
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--roll", LaunchConfiguration("camera_roll"),
            "--pitch", LaunchConfiguration("camera_pitch"),
            "--yaw", LaunchConfiguration("camera_yaw"),
            "--frame-id", "base_link",
            "--child-frame-id", "nav_camera_link",
        ],
        output="screen",
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "online.launch.py")),
        launch_arguments={
            "start_camera": "true",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": "false",
            "camera_namespace": "sensors",
            "camera_name": "nav_camera",
            "serial_no": LaunchConfiguration("serial_no"),
            "camera_stage": "aligned",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "odom_source": "external",
            "odom_topic": "/swerve_controller/odom",
            "map_frame": "map",
            "publish_odom_tf": "false",
            "publish_map_tf": "true",
            "mode": "mapping",
            "fresh_map": LaunchConfiguration("fresh_map"),
            "database_path": LaunchConfiguration("database_path"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no", default_value="",
            description="D435i serial number; required when multiple cameras are connected",
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/mobile_base_online.db"
        ),
        # Temporary measured-mount placeholders. Replace with calibrated values.
        DeclareLaunchArgument("camera_x", default_value="0.10"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="1.12"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.20944"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        camera_mount,
        slam,
    ])
