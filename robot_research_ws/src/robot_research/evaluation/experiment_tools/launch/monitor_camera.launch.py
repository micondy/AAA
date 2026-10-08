from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from experiment_tools.camera_topics import camera_prefix, depth_topic


def _create_monitor(context):
    namespace = LaunchConfiguration("camera_namespace").perform(context)
    name = LaunchConfiguration("camera_name").perform(context)
    stage = LaunchConfiguration("stage").perform(context)
    prefix = camera_prefix(namespace, name)
    return [Node(
        package="experiment_tools",
        executable="camera_health_monitor",
        parameters=[{
            "color_topic": f"{prefix}/color/image_raw",
            "depth_topic": depth_topic(namespace, name, stage),
            "imu_topic": f"{prefix}/imu",
            "require_imu": stage.strip().lower() != "basic",
        }],
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument(
            "stage", default_value="full",
            description="Camera feature stage: basic, imu, aligned, or full",
        ),
        OpaqueFunction(function=_create_monitor),
    ])
