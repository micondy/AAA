from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_navigation_bringup"))
    rtabmap_share = Path(get_package_share_directory("rtabmap_bringup"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={
                "use_sim_time": use_sim_time,
                "xy_goal_tolerance": LaunchConfiguration("xy_goal_tolerance"),
                "yaw_goal_tolerance": LaunchConfiguration("yaw_goal_tolerance"),
                "required_movement_radius": LaunchConfiguration(
                    "required_movement_radius"
                ),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    costmap_overlay = ParameterFile(
        LaunchConfiguration("costmap_overlay_file"),
        allow_substs=True,
    )
    common = {
        # The second file is deliberately an overlay.  It only changes the
        # costmap plugin lists and adds plugin-specific parameters; all motion,
        # safety and localization settings continue to come from nav2.yaml.
        "parameters": [configured_params, costmap_overlay],
        "output": "screen",
        "respawn": LaunchConfiguration("use_respawn"),
        "respawn_delay": 2.0,
    }

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rtabmap_share / "launch" / "online.launch.py")
        ),
        launch_arguments={
            "database_path": LaunchConfiguration("database_path"),
            "start_camera": LaunchConfiguration("start_camera"),
            "use_rviz": "false",
            "use_sim_time": use_sim_time,
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "camera_name": LaunchConfiguration("camera_name"),
            "serial_no": LaunchConfiguration("serial_no"),
            "camera_stage": "full",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "map_frame": "map",
            "initial_pose": LaunchConfiguration("initial_pose"),
            "mode": LaunchConfiguration("rtabmap_mode"),
            "fresh_map": LaunchConfiguration("fresh_map"),
        }.items(),
    )
    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="nav_camera_mount_tf",
        arguments=[
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--roll", LaunchConfiguration("camera_roll"),
            "--pitch", LaunchConfiguration("camera_pitch"),
            "--yaw", LaunchConfiguration("camera_yaw"),
            "--frame-id", "base_link",
            "--child-frame-id", "nav_camera_link",
        ],
        condition=IfCondition(LaunchConfiguration("publish_camera_mount_tf")),
        output="screen",
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        remappings=[("cmd_vel", "/cmd_vel_nav")],
        **common,
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        **common,
    )
    path_smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        **common,
    )
    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        remappings=[("cmd_vel", "/cmd_vel_nav")],
        **common,
    )
    navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[
            configured_params,
            {
                "default_nav_to_pose_bt_xml": str(
                    share / "config" / "navigate_to_pose_safe.xml"
                )
            },
        ],
        output="screen",
        respawn=LaunchConfiguration("use_respawn"),
        respawn_delay=2.0,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", LaunchConfiguration("cmd_vel_smoothed_topic")),
        ],
        **common,
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        parameters=[
            configured_params,
            costmap_overlay,
            {
                "cmd_vel_in_topic": LaunchConfiguration("cmd_vel_in_topic"),
                "cmd_vel_out_topic": LaunchConfiguration("cmd_vel_out_topic"),
                "source_timeout": LaunchConfiguration("collision_source_timeout"),
            },
        ],
        output="screen",
        respawn=LaunchConfiguration("use_respawn"),
        respawn_delay=2.0,
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "autostart": autostart,
            "node_names": [
                "controller_server",
                "planner_server",
                "smoother_server",
                "behavior_server",
                "bt_navigator",
                "velocity_smoother",
                "collision_monitor",
            ],
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(nav2_share / "rviz" / "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_respawn", default_value="false"),
        DeclareLaunchArgument(
            "navigation_startup_delay",
            default_value="5.0",
            description="Seconds reserved for localization TF before Nav2 activation",
        ),
        DeclareLaunchArgument(
            "cmd_vel_in_topic",
            default_value="/cmd_vel_smoothed",
            description="Velocity input to Collision Monitor",
        ),
        DeclareLaunchArgument(
            "cmd_vel_out_topic",
            default_value="/swerve_controller/cmd_vel",
            description="Safe velocity output; use /cmd_vel for Nova Carter",
        ),
        DeclareLaunchArgument(
            "cmd_vel_smoothed_topic",
            default_value="/cmd_vel_smoothed",
            description="Velocity smoother output topic before collision monitoring",
        ),
        DeclareLaunchArgument(
            "collision_source_timeout",
            default_value="0.75",
            description="Maximum sensor age accepted by collision_monitor",
        ),
        DeclareLaunchArgument("xy_goal_tolerance", default_value="0.15"),
        DeclareLaunchArgument("yaw_goal_tolerance", default_value="0.15"),
        DeclareLaunchArgument("required_movement_radius", default_value="0.15"),
        DeclareLaunchArgument(
            "initial_pose",
            default_value="",
            description="Optional RTAB-Map startup pose: x y z roll pitch yaw",
        ),
        DeclareLaunchArgument(
            "rtabmap_mode",
            default_value="localization",
            description="RTAB-Map mode: localization or online incremental mapping",
        ),
        DeclareLaunchArgument(
            "fresh_map",
            default_value="false",
            description="Delete the selected database before online mapping starts",
        ),
        DeclareLaunchArgument("camera_namespace", default_value="sensors"),
        DeclareLaunchArgument("camera_name", default_value="nav_camera"),
        DeclareLaunchArgument("odom_source", default_value="rgbd"),
        DeclareLaunchArgument("odom_topic", default_value="/swerve_controller/odom"),
        DeclareLaunchArgument("publish_camera_mount_tf", default_value="false"),
        DeclareLaunchArgument("camera_x", default_value=""),
        DeclareLaunchArgument("camera_y", default_value=""),
        DeclareLaunchArgument("camera_z", default_value=""),
        DeclareLaunchArgument("camera_roll", default_value=""),
        DeclareLaunchArgument("camera_pitch", default_value=""),
        DeclareLaunchArgument("camera_yaw", default_value=""),
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description="Serial number of the fixed D435i; required in a multi-camera setup",
        ),
        DeclareLaunchArgument(
            "database_path",
            description="Existing RTAB-Map database created by mapping.launch.py",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=str(share / "config" / "nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "costmap_overlay_file",
            default_value=str(share / "config" / "costmap_baseline_overlay.yaml"),
            description="Optional costmap-only parameter overlay",
        ),
        camera_mount_tf,
        rtabmap,
        controller,
        planner,
        path_smoother,
        behaviors,
        navigator,
        velocity_smoother,
        collision_monitor,
        TimerAction(
            period=LaunchConfiguration("navigation_startup_delay"),
            actions=[lifecycle],
        ),
        rviz,
    ])
