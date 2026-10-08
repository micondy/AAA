from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("gaussian_splatting_bringup"))
    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "capture.launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "global_frame": LaunchConfiguration("global_frame"),
            "output_dir": LaunchConfiguration("output_dir"),
            "maximum_frames": LaunchConfiguration("maximum_frames"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("global_frame", default_value="odom"),
        DeclareLaunchArgument("output_dir", default_value="gaussian_dataset_real"),
        DeclareLaunchArgument("maximum_frames", default_value="0"),
        capture,
    ])
