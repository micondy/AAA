"""One-command three-layer rack perception and alignment validation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _check_yolo(context):
    if LaunchConfiguration("use_yolo").perform(context).lower() not in {"true", "1"}:
        return []
    if LaunchConfiguration("anonymous_instances").perform(context).lower() not in {"true", "1"}:
        raise RuntimeError("use_yolo:=true requires anonymous_instances:=true for ID-free rack_map")
    weights = Path(LaunchConfiguration("yolo_weights_path").perform(context)).expanduser()
    if not weights.is_file():
        raise RuntimeError(f"trained bed/post weights not found: {weights}")
    return []


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("mobile_manipulator_bringup"))
    rack_share = Path(get_package_share_directory("rack_perception"))
    nvblox_share = Path(get_package_share_directory("nvblox_bringup"))
    config = str(rack_share / "config" / "rack_sim.yaml")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "sim_perception.launch.py")),
        launch_arguments={
            "start_isaac": LaunchConfiguration("start_isaac"),
            "headless": LaunchConfiguration("headless"),
            "empty_scene": "true",
            "test_seconds": LaunchConfiguration("test_seconds"),
            "add_green_target": "false",
            "add_rack": "true",
            "rack_asset": LaunchConfiguration("rack_asset"),
            "rack_layout": LaunchConfiguration("rack_layout"),
            "rack_visual_seed": LaunchConfiguration("rack_visual_seed"),
            "parametric_rack": LaunchConfiguration("parametric_rack"),
            "start_target_perception": "false",
            "enable_base_control": "true",
            "output_frame": "odom",
        }.items(),
    )

    nodes = [
        Node(
            package="rack_perception", executable="rack_detector", name="rack_detector",
            output="screen", parameters=[config, {
                "anonymous_instances": ParameterValue(
                    LaunchConfiguration("anonymous_instances"), value_type=bool
                ),
            }], condition=UnlessCondition(LaunchConfiguration("use_yolo")),
        ),
        ExecuteProcess(
            cmd=[
                LaunchConfiguration("yolo_python_executable"), "-m", "rack_perception.yolo_detector_node",
                "--ros-args", "-p", "use_sim_time:=true",
                "-p", ["weights_path:=", LaunchConfiguration("yolo_weights_path")],
            ],
            output="screen", condition=IfCondition(LaunchConfiguration("use_yolo")),
        ),
        Node(package="rack_perception", executable="rack_geometry", name="rack_geometry",
             output="screen", parameters=[config]),
        Node(package="rack_perception", executable="rack_map", name="rack_map",
             output="screen", parameters=[config],
             condition=UnlessCondition(LaunchConfiguration("anonymous_instances"))),
        Node(package="rack_perception", executable="rack_map", name="rack_map",
             output="screen", parameters=[config, {
                 "use_source_instance_hints": False,
                 "rack_distance_m": 1.8,
                 "group_distance_m": 1.1,
                 "layer_xy_distance_m": 1.8,
                 "min_confirmed_frames": 3,
             }], condition=IfCondition(LaunchConfiguration("anonymous_instances"))),
        Node(package="rack_perception", executable="harvest_planner", name="harvest_planner",
             output="screen", parameters=[config]),
        Node(package="rack_perception", executable="rack_alignment", name="rack_alignment",
             output="screen", parameters=[config]),
    ]
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
        condition=IfCondition(LaunchConfiguration("enable_nvblox")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_isaac", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("test_seconds", default_value="0"),
            DeclareLaunchArgument("enable_nvblox", default_value="false"),
            DeclareLaunchArgument("rack_asset", default_value=""),
            DeclareLaunchArgument("rack_layout", default_value="grid4x4"),
            DeclareLaunchArgument("rack_visual_seed", default_value="0"),
            DeclareLaunchArgument("parametric_rack", default_value="false"),
            DeclareLaunchArgument("anonymous_instances", default_value="false"),
            DeclareLaunchArgument("use_yolo", default_value="false"),
            DeclareLaunchArgument("yolo_weights_path", default_value=""),
            DeclareLaunchArgument("yolo_python_executable", default_value="python3"),
            OpaqueFunction(function=_check_yolo),
            simulator,
            nvblox,
            *nodes,
        ]
    )
