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
            "use_sim_time": "true",
            "color_topic": "/sensors/nav_camera/color/image_raw",
            "depth_topic": "/sensors/nav_camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/sensors/nav_camera/color/camera_info",
            "global_frame": "odom",
            "output_dir": LaunchConfiguration("output_dir"),
            "maximum_frames": LaunchConfiguration("maximum_frames"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="gaussian_dataset_sim"),
        DeclareLaunchArgument("maximum_frames", default_value="0"),
        capture,
    ])
