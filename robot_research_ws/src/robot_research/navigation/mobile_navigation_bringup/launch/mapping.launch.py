from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rtabmap_share = Path(get_package_share_directory("rtabmap_bringup"))
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rtabmap_share / "launch" / "online.launch.py")
        ),
        launch_arguments={
            "start_camera": LaunchConfiguration("start_camera"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "camera_name": LaunchConfiguration("camera_name"),
            "serial_no": LaunchConfiguration("serial_no"),
            "camera_stage": "aligned",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "map_frame": "map",
            "publish_odom_tf": "true",
            "publish_map_tf": "true",
            "mode": "mapping",
            "fresh_map": LaunchConfiguration("fresh_map"),
            "database_path": LaunchConfiguration("database_path"),
        }.items(),
    )
    camera_mount_tf = Node(
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
        condition=IfCondition(LaunchConfiguration("publish_camera_mount_tf")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("camera_namespace", default_value="sensors"),
        DeclareLaunchArgument("camera_name", default_value="nav_camera"),
        DeclareLaunchArgument("odom_source", default_value="rgbd"),
        DeclareLaunchArgument("odom_topic", default_value="/swerve_controller/odom"),
        DeclareLaunchArgument("publish_camera_mount_tf", default_value="false"),
        DeclareLaunchArgument("camera_x", default_value=""),
        DeclareLaunchArgument("camera_y", default_value=""),
        DeclareLaunchArgument("camera_z", default_value=""),
        DeclareLaunchArgument("camera_roll", default_value=""),
        DeclareLaunchArgument("camera_pitch", default_value=""),
        DeclareLaunchArgument("camera_yaw", default_value=""),
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description="Serial number of the fixed D435i; required in a multi-camera setup",
        ),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/mobile_robot_map.db"
        ),
        camera_mount_tf,
        mapping,
    ])
