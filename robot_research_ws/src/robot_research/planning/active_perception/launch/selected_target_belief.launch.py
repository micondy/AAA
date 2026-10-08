from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("active_perception")
    selector_config = share + "/config/target_selector.yaml"
    belief_config = share + "/config/belief.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("belief_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "detection_topic",
                default_value="/target_perception/tracked_detections_3d",
            ),
            DeclareLaunchArgument("target_id", default_value=""),
            DeclareLaunchArgument("target_class", default_value="*"),
            DeclareLaunchArgument("max_missed_frames", default_value="5"),
            Node(
                package="active_perception",
                executable="target_selector",
                name="target_selector",
                output="screen",
                parameters=[
                    selector_config,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_topic": LaunchConfiguration("detection_topic"),
                        "target_id": LaunchConfiguration("target_id"),
                        "target_class": LaunchConfiguration("target_class"),
                        "max_missed_frames": LaunchConfiguration(
                            "max_missed_frames"
                        ),
                    },
                ],
            ),
            Node(
                package="active_perception",
                executable="target_belief",
                name="target_belief",
                output="screen",
                parameters=[
                    belief_config,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "belief_frame": LaunchConfiguration("belief_frame"),
                        "input_topic": "/active_perception/selected_target_pose",
                    },
                ],
            ),
        ]
    )
