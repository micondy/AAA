from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = Path(get_package_share_directory("mobile_navigation_bringup"))
    description_share = Path(get_package_share_directory("mobile_manipulator_description"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    nvblox_share = Path(get_package_share_directory("nvblox_bringup"))

    def launch_rviz(context):
        # Evaluate before nested launch files set their own `use_rviz=false`.
        enabled = LaunchConfiguration("use_rviz").perform(context).lower()
        if enabled not in ("true", "1", "yes"):
            return []
        return [Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(nav2_share / "rviz" / "nav2_default_view.rviz")],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )]

    description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(description_share / "launch" / "display.launch.py")
            ),
            launch_arguments={
                "use_rviz": "false",
                "use_joint_state_publisher": "false",
                "use_sim_time": "true",
                "nav_camera_use_nominal_extrinsics": "true",
            }.items(),
    )

    def navigation_with_overlay(overlay_name, condition):
        return IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(navigation_share / "launch" / "navigation.launch.py")
                ),
                launch_arguments={
                "start_camera": "false",
                "use_sim_time": "true",
                "use_rviz": "false",
                "database_path": LaunchConfiguration("database_path"),
                "rtabmap_mode": LaunchConfiguration("rtabmap_mode"),
                "fresh_map": LaunchConfiguration("fresh_map"),
                "cmd_vel_out_topic": "/cmd_vel_nav_safe",
                # Keep the normal Nav2 command chain in simulation:
                # controller -> velocity smoother -> collision monitor -> Isaac.
                "cmd_vel_smoothed_topic": "/cmd_vel_smoothed",
                # Isaac can pause RGB-D publication briefly while rendering. This
                # relaxed timeout is simulation-only; the real profile keeps 0.75 s.
                "collision_source_timeout": "2.0",
                # Keep web-click goals precise enough that a nearby click still
                # causes visible motion instead of being accepted immediately.
                "xy_goal_tolerance": "0.08",
                "yaw_goal_tolerance": "0.15",
                "required_movement_radius": "0.08",
                "initial_pose": LaunchConfiguration("initial_pose"),
                "odom_source": "external",
                "odom_topic": "/swerve_controller/odom",
                "costmap_overlay_file": str(
                    navigation_share / "config" / overlay_name
                ),
                }.items(),
                condition=condition,
        )

    use_nvblox_costmap = LaunchConfiguration("use_nvblox_costmap")
    navigation_baseline = navigation_with_overlay(
        "costmap_baseline_overlay.yaml", UnlessCondition(use_nvblox_costmap)
    )
    navigation_nvblox = navigation_with_overlay(
        "costmap_nvblox_overlay.yaml", IfCondition(use_nvblox_costmap)
    )
    nvblox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nvblox_share / "launch" / "mapping.launch.py")
            ),
            launch_arguments={
                "start_camera": "false",
                "use_rviz": "false",
                "use_sim_time": "true",
                "global_frame": "odom",
                "base_frame": "base_link",
                "input_qos": "DEFAULT",
            }.items(),
            # The Nav2 nvblox profile cannot operate without its map producer.
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("enable_nvblox"),
                    "'.lower() in ('true', '1', 'yes') or '",
                    use_nvblox_costmap,
                    "'.lower() in ('true', '1', 'yes')",
                ])
            ),
    )
    web_console = Node(
        package="mobile_base_web_console",
        executable="web_console",
        name="mobile_base_web_console",
        output="screen",
        remappings=[
            ("/swerve_controller/cmd_vel", "/cmd_vel_web"),
            ("/map", "/rtabmap/map"),
        ],
        additional_env={"MOBILE_BASE_SIMULATION_DIRECT_CONTROL": "true"},
        condition=IfCondition(LaunchConfiguration("start_web_console")),
    )
    command_mux = Node(
        package="mobile_navigation_bringup",
        executable="cmd_vel_priority_mux.py",
        name="cmd_vel_priority_mux",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("enable_nvblox", default_value="false"),
        DeclareLaunchArgument(
            "start_web_console",
            default_value="true",
            description="Start the LAN web map, goal and 3D visualization console",
        ),
        DeclareLaunchArgument(
            "use_nvblox_costmap",
            default_value="false",
            description=(
                "Use the optional nvblox ESDF costmap overlay; this also starts nvblox"
            ),
        ),
        DeclareLaunchArgument(
            "database_path",
            description="Existing custom mobile-base RTAB-Map database",
        ),
        DeclareLaunchArgument(
            "rtabmap_mode",
            default_value="localization",
            description="Use mapping for online map growth or localization for a fixed map",
        ),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument(
            "initial_pose",
            default_value="0 0 0 0 0 0",
            description="Map pose of the deterministic Isaac odometry origin",
        ),
        OpaqueFunction(function=launch_rviz),
        description,
        navigation_baseline,
        navigation_nvblox,
        nvblox,
        command_mux,
        web_console,
    ])
