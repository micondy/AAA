from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    recorder = Node(
        package="gaussian_splatting_bringup",
        executable="dataset_recorder",
        name="gaussian_splatting_dataset_recorder",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            "color_topic": ParameterValue(LaunchConfiguration("color_topic"), value_type=str),
            "depth_topic": ParameterValue(LaunchConfiguration("depth_topic"), value_type=str),
            "camera_info_topic": ParameterValue(LaunchConfiguration("camera_info_topic"), value_type=str),
            "global_frame": ParameterValue(LaunchConfiguration("global_frame"), value_type=str),
            "output_dir": ParameterValue(LaunchConfiguration("output_dir"), value_type=str),
            "minimum_interval_s": ParameterValue(LaunchConfiguration("minimum_interval_s"), value_type=float),
            "minimum_translation_m": ParameterValue(LaunchConfiguration("minimum_translation_m"), value_type=float),
            "minimum_rotation_deg": ParameterValue(LaunchConfiguration("minimum_rotation_deg"), value_type=float),
            "maximum_frames": ParameterValue(LaunchConfiguration("maximum_frames"), value_type=int),
            "tf_timeout_s": ParameterValue(LaunchConfiguration("tf_timeout_s"), value_type=float),
            "pose_delay_s": ParameterValue(LaunchConfiguration("pose_delay_s"), value_type=float),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("color_topic", default_value="/sensors/nav_camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/sensors/nav_camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/sensors/nav_camera/color/camera_info"),
        DeclareLaunchArgument("global_frame", default_value="odom"),
        DeclareLaunchArgument("output_dir", default_value="gaussian_dataset"),
        DeclareLaunchArgument("minimum_interval_s", default_value="0.15"),
        DeclareLaunchArgument("minimum_translation_m", default_value="0.03"),
        DeclareLaunchArgument("minimum_rotation_deg", default_value="3.0"),
        DeclareLaunchArgument("maximum_frames", default_value="0"),
        DeclareLaunchArgument("tf_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("pose_delay_s", default_value="0.25"),
        recorder,
    ])
