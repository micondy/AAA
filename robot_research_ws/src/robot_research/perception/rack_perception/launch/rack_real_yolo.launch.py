"""Real D435i rack perception; does not start or alter navigation."""

from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_inputs(context):
    raw_weights = LaunchConfiguration("weights_path").perform(context).strip()
    if not raw_weights:
        raise RuntimeError("weights_path must point to custom-trained bed/post best.pt")
    weights = Path(raw_weights).expanduser()
    if not weights.is_file():
        raise RuntimeError(f"trained bed/post weights are required: {weights}")
    python = LaunchConfiguration("python_executable").perform(context)
    if not (Path(python).is_file() or shutil.which(python)):
        raise RuntimeError(f"Python executable not found: {python}")
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("rack_perception"))
    params = str(share / "config" / "rack_real_yolo.yaml")
    detector = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"), "-m", "rack_perception.yolo_detector_node",
            "--ros-args", "--params-file", params,
            "-p", ["weights_path:=", LaunchConfiguration("weights_path")],
        ],
        output="screen",
    )
    geometry = Node(
        package="rack_perception", executable="rack_geometry", name="rack_geometry",
        parameters=[params, {"output_frame": LaunchConfiguration("output_frame")}],
        condition=IfCondition(LaunchConfiguration("start_geometry")), output="screen",
    )
    rack_map = Node(
        package="rack_perception", executable="rack_map", name="rack_map",
        parameters=[params], condition=IfCondition(LaunchConfiguration("start_map")),
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("python_executable", default_value="python3"),
        DeclareLaunchArgument("weights_path", default_value=""),
        DeclareLaunchArgument("output_frame", default_value="odom"),
        DeclareLaunchArgument("start_geometry", default_value="true"),
        DeclareLaunchArgument("start_map", default_value="true"),
        OpaqueFunction(function=_validate_inputs),
        detector, geometry, rack_map,
    ])
