from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory("active_perception") + "/config/belief.yaml"
    benchmark = Node(
        package="active_perception",
        executable="belief_benchmark",
        name="belief_benchmark",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_benchmark")),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "expected_frame": LaunchConfiguration("belief_frame"),
                "output_csv": ParameterValue(
                    LaunchConfiguration("benchmark_output_csv"), value_type=str
                ),
                "sample_count": ParameterValue(
                    LaunchConfiguration("benchmark_samples"), value_type=int
                ),
                "warmup_frames": ParameterValue(
                    LaunchConfiguration("benchmark_warmup_frames"), value_type=int
                ),
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("belief_frame", default_value="base_link"),
            DeclareLaunchArgument("run_benchmark", default_value="false"),
            DeclareLaunchArgument("benchmark_samples", default_value="100"),
            DeclareLaunchArgument("benchmark_warmup_frames", default_value="15"),
            DeclareLaunchArgument(
                "benchmark_output_csv", default_value="/tmp/target_belief.csv"
            ),
            Node(
                package="active_perception",
                executable="target_belief",
                name="target_belief",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "belief_frame": LaunchConfiguration("belief_frame"),
                    },
                ],
            ),
            benchmark,
        ]
    )
