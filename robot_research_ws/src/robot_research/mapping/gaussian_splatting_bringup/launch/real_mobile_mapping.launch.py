"""Real mobile RGB-D mapping with one TF tree and shared camera streams.

TF ownership is deliberately split as follows:

* robot_state_publisher: base_link -> nav_camera_link (measured mount)
* realsense2_camera: nav_camera_link -> calibrated optical frames
* RTAB-Map RGB-D odometry or the base driver: odom -> base_link
* RTAB-Map SLAM: map -> odom

Nvblox and the Gaussian recorder are consumers only.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gaussian_share = Path(get_package_share_directory("gaussian_splatting_bringup"))
    description_share = Path(get_package_share_directory("mobile_manipulator_description"))
    nvblox_share = Path(get_package_share_directory("nvblox_bringup"))
    rtabmap_share = Path(get_package_share_directory("rtabmap_bringup"))
    xacro_file = description_share / "urdf" / "mobile_manipulator.urdf.xacro"

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ", str(xacro_file),
                " use_mock_hardware:=true",
                " openarm_ros2_control:=false",
                " nav_camera_enabled:=true",
                " nav_camera_use_nominal_extrinsics:=false",
                " nav_camera_mount_xyz:='", LaunchConfiguration("camera_mount_xyz"), "'",
                " nav_camera_mount_rpy:='", LaunchConfiguration("camera_mount_rpy"), "'",
            ]),
            value_type=str,
        )
    }
    description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="mobile_manipulator_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
        output="screen",
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(rtabmap_share / "launch" / "online.launch.py")),
        launch_arguments={
            "start_camera": LaunchConfiguration("start_camera"),
            "use_rviz": "false",
            "use_sim_time": "false",
            "camera_namespace": "sensors",
            "camera_name": "nav_camera",
            "serial_no": LaunchConfiguration("serial_no"),
            "camera_stage": "aligned",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "map_frame": "map",
            "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
            "publish_map_tf": "true",
            "mode": "mapping",
            "fresh_map": LaunchConfiguration("fresh_map"),
            "database_path": LaunchConfiguration("database_path"),
        }.items(),
    )

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nvblox_share / "launch" / "mapping.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_nvblox")),
        launch_arguments={
            "start_camera": "false",
            "use_rviz": "false",
            "use_sim_time": "false",
            "camera_namespace": "sensors",
            "camera_name": "nav_camera",
            "global_frame": "odom",
            "base_frame": "base_link",
            "input_qos": "SENSOR_DATA",
        }.items(),
    )

    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gaussian_share / "launch" / "capture.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_gaussian_capture")),
        launch_arguments={
            "use_sim_time": "false",
            "color_topic": "/sensors/nav_camera/color/image_raw",
            "depth_topic": "/sensors/nav_camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/sensors/nav_camera/color/camera_info",
            "global_frame": "odom",
            "output_dir": LaunchConfiguration("output_dir"),
            "maximum_frames": LaunchConfiguration("maximum_frames"),
            "minimum_interval_s": LaunchConfiguration("minimum_interval_s"),
            "minimum_translation_m": LaunchConfiguration("minimum_translation_m"),
            "minimum_rotation_deg": LaunchConfiguration("minimum_rotation_deg"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="real_mobile_mapping_rviz",
        arguments=["-d", str(nvblox_share / "rviz" / "nvblox.rviz")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value="346122070466"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_nvblox", default_value="true"),
        DeclareLaunchArgument("start_gaussian_capture", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "odom_source",
            default_value="rgbd",
            description="Use rgbd now; select external only when base odometry is live.",
        ),
        DeclareLaunchArgument("odom_topic", default_value="/swerve_controller/odom"),
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description="Set false only if an external base driver already owns odom->base_link.",
        ),
        DeclareLaunchArgument("camera_mount_xyz", default_value="0.10 0 1.12"),
        DeclareLaunchArgument("camera_mount_rpy", default_value="0 0.20944 0"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/real_mobile_gaussian_map.db"
        ),
        DeclareLaunchArgument("output_dir", default_value="gaussian_dataset_real_mobile"),
        DeclareLaunchArgument("maximum_frames", default_value="0"),
        DeclareLaunchArgument("minimum_interval_s", default_value="0.15"),
        DeclareLaunchArgument("minimum_translation_m", default_value="0.03"),
        DeclareLaunchArgument("minimum_rotation_deg", default_value="3.0"),
        description,
        slam,
        nvblox,
        capture,
        rviz,
    ])
