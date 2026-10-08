"""A3 integration profile: custom base Nav2 plus fixed-camera NBV planning."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("mobile_manipulator_bringup"))
    navigation_share = Path(get_package_share_directory("mobile_navigation_bringup"))
    perception_share = Path(get_package_share_directory("target_perception"))
    active_share = Path(get_package_share_directory("active_perception"))
    workspace = bringup_share.parents[3]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "mobile_base_sim_navigation.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "database_path": LaunchConfiguration("database_path"),
            "initial_pose": LaunchConfiguration("initial_pose"),
        }.items(),
    )
    detector = Node(
        package="target_perception",
        executable="color_depth_detector",
        name="color_depth_detector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_target_perception")),
        parameters=[
            str(perception_share / "config" / "color_depth.yaml"),
            {
                # Track task objects in the continuous odom frame. Base-frame
                # coordinates change during navigation and create false IDs.
                "output_frame": "odom",
                "use_sim_time": True,
                # The generic room also contains a desaturated green bench.
                # Restrict only this synthetic profile to the pure-green cube;
                # keep the reusable detector configuration broad for real data.
                "hsv_lower": [50, 150, 80],
                "hsv_upper": [63, 255, 255],
            },
        ],
    )
    tracker = Node(
        package="target_perception",
        executable="instance_tracking",
        name="instance_tracking",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_target_perception")),
        parameters=[
            str(perception_share / "config" / "instance_tracking.yaml"),
            {
                "use_sim_time": True,
                # Cover the executor's 120 s navigation window at 10 Hz.
                "max_missed_frames": 1500,
            },
        ],
    )
    active_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(active_share / "launch" / "active_perception_stack.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            # RTAB-Map publishes map->odom after processing each RGB-D frame.
            # Fuse observations in continuous odom and transform once, at NBV
            # trigger time, instead of requesting a future map TF per frame.
            "belief_frame": "odom",
            "detection_topic": "/target_perception/tracked_detections_3d",
            "map_topic": "/rtabmap/map",
            "target_id": LaunchConfiguration("target_id"),
            "target_class": LaunchConfiguration("target_class"),
            "target_max_missed_frames": "1500",
            "execution_enabled": LaunchConfiguration("enable_nbv_execution"),
            "execution_output_directory": str(
                workspace / "results" / "active_perception" / "executions"
            ),
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="active_perception_rviz",
        arguments=[
            "-d",
            str(bringup_share / "rviz" / "active_perception_navigation.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value=str(
                    workspace / "maps" / "isaac_indoor_external_odom_04.db"
                ),
                description="Existing RTAB-Map database for the custom-base scene",
            ),
            DeclareLaunchArgument("initial_pose", default_value="0 0 0 0 0 0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_target_perception", default_value="true"),
            DeclareLaunchArgument("target_id", default_value=""),
            DeclareLaunchArgument("target_class", default_value="green_target"),
            DeclareLaunchArgument("enable_nbv_execution", default_value="false"),
            navigation,
            detector,
            tracker,
            active_perception,
            rviz,
        ]
    )
