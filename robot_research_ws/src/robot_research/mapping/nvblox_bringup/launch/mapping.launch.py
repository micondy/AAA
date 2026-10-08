"""Optional single-RGB-D nvblox mapping profile.

This launch deliberately owns neither odometry nor map localization. A valid TF
from ``global_frame`` to the camera optical frame must already exist.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _camera_prefix(namespace: str, name: str) -> str:
    namespace = namespace.strip("/")
    name = name.strip("/")
    if not namespace or not name or "/" in name:
        raise RuntimeError("camera_namespace and camera_name must be non-empty ROS names")
    return f"/{namespace}/{name}"


def _make_nvblox(context):
    camera = _camera_prefix(
        LaunchConfiguration("camera_namespace").perform(context),
        LaunchConfiguration("camera_name").perform(context),
    )
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    parameters = [
        ParameterFile(LaunchConfiguration("params_file"), allow_substs=True),
        {
            "use_sim_time": use_sim_time,
            "global_frame": ParameterValue(
                LaunchConfiguration("global_frame"), value_type=str
            ),
            "map_clearing_frame_id": ParameterValue(
                LaunchConfiguration("base_frame"), value_type=str
            ),
            "esdf_slice_bounds_visualization_attachment_frame_id": ParameterValue(
                LaunchConfiguration("base_frame"), value_type=str
            ),
            "workspace_height_bounds_visualization_attachment_frame_id": ParameterValue(
                LaunchConfiguration("base_frame"), value_type=str
            ),
            "input_qos": ParameterValue(
                LaunchConfiguration("input_qos"), value_type=str
            ),
        },
    ]
    remappings = [
        ("camera_0/color/image", f"{camera}/color/image_raw"),
        ("camera_0/color/camera_info", f"{camera}/color/camera_info"),
        (
            "camera_0/depth/image",
            f"{camera}/aligned_depth_to_color/image_raw",
        ),
        ("camera_0/depth/camera_info", f"{camera}/color/camera_info"),
    ]
    component = ComposableNode(
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        name="nvblox_node",
        parameters=parameters,
        remappings=remappings,
        extra_arguments=[{"use_intra_process_comms": False}],
    )
    return [
        ComposableNodeContainer(
            name="nvblox_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[component],
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("nvblox_bringup"))
    camera_share = Path(get_package_share_directory("d435i_bringup"))
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / "launch" / "camera.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz": "false",
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "camera_name": LaunchConfiguration("camera_name"),
            "serial_no": LaunchConfiguration("serial_no"),
            "stage": "aligned",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(share / "rviz" / "nvblox.rviz")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("camera_namespace", default_value="sensors"),
            DeclareLaunchArgument("camera_name", default_value="nav_camera"),
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("global_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "input_qos",
                default_value="SENSOR_DATA",
                description="Use DEFAULT for the current Isaac Sim publishers",
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config" / "mapping.yaml")
            ),
            camera,
            OpaqueFunction(function=_make_nvblox),
            rviz,
        ]
    )
