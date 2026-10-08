from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_directory = get_package_share_directory("active_perception") + "/launch"
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("belief_frame", default_value="map"),
            DeclareLaunchArgument(
                "detection_topic",
                default_value="/target_perception/tracked_detections_3d",
            ),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("start_executor", default_value="true"),
            DeclareLaunchArgument("execution_enabled", default_value="false"),
            DeclareLaunchArgument(
                "execution_output_directory",
                default_value="/tmp/active_perception_executions",
            ),
            DeclareLaunchArgument("target_id", default_value=""),
            DeclareLaunchArgument("target_class", default_value="*"),
            DeclareLaunchArgument("target_max_missed_frames", default_value="5"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_directory + "/selected_target_belief.launch.py"
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "belief_frame": LaunchConfiguration("belief_frame"),
                    "detection_topic": LaunchConfiguration("detection_topic"),
                    "target_id": LaunchConfiguration("target_id"),
                    "target_class": LaunchConfiguration("target_class"),
                    "max_missed_frames": LaunchConfiguration(
                        "target_max_missed_frames"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_directory + "/nbv_planner.launch.py"
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map_topic": LaunchConfiguration("map_topic"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_directory + "/nbv_executor.launch.py"
                ),
                condition=IfCondition(LaunchConfiguration("start_executor")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "execution_enabled": LaunchConfiguration("execution_enabled"),
                    "output_directory": LaunchConfiguration(
                        "execution_output_directory"
                    ),
                }.items(),
            ),
        ]
    )
