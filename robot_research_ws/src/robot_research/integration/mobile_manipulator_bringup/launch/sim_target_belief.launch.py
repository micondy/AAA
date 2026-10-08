"""Isaac RGB-D target observations feeding the target-belief filter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("mobile_manipulator_bringup"))
    belief_share = Path(get_package_share_directory("active_perception"))
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "sim_perception.launch.py")
        ),
        launch_arguments={
            "start_isaac": LaunchConfiguration("start_isaac"),
            "headless": LaunchConfiguration("headless"),
            "test_seconds": LaunchConfiguration("test_seconds"),
            "green_target_depth": LaunchConfiguration("green_target_depth"),
            "output_frame": "base_link",
            "run_benchmark": "false",
            "run_distance_benchmark": "false",
        }.items(),
    )
    belief = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(belief_share / "launch" / "belief.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "belief_frame": LaunchConfiguration("belief_frame"),
            "run_benchmark": LaunchConfiguration("run_belief_benchmark"),
            "benchmark_samples": LaunchConfiguration("benchmark_samples"),
            "benchmark_warmup_frames": LaunchConfiguration(
                "benchmark_warmup_frames"
            ),
            "benchmark_output_csv": LaunchConfiguration("benchmark_output_csv"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_isaac", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("test_seconds", default_value="0"),
            DeclareLaunchArgument("green_target_depth", default_value="0.77"),
            DeclareLaunchArgument("belief_frame", default_value="base_link"),
            DeclareLaunchArgument("run_belief_benchmark", default_value="false"),
            DeclareLaunchArgument("benchmark_samples", default_value="100"),
            DeclareLaunchArgument("benchmark_warmup_frames", default_value="15"),
            DeclareLaunchArgument(
                "benchmark_output_csv", default_value="/tmp/target_belief.csv"
            ),
            perception,
            belief,
        ]
    )
