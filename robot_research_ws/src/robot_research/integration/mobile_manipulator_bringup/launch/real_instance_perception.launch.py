"""One-command D435i plus general RGB-D instance-perception profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("mobile_manipulator_bringup"))
    camera_share = Path(get_package_share_directory("d435i_bringup"))
    experiment_share = Path(get_package_share_directory("experiment_tools"))
    perception_share = Path(get_package_share_directory("target_perception"))
    workspace = bringup_share.parents[3]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / "launch" / "camera.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_camera")),
        launch_arguments={
            "camera_namespace": "sensors",
            "camera_name": "nav_camera",
            "serial_no": LaunchConfiguration("serial_no"),
            "stage": LaunchConfiguration("camera_stage"),
            "reset_on_start": LaunchConfiguration("reset_on_start"),
            "use_rviz": "false",
            "use_sim_time": "false",
        }.items(),
    )
    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(experiment_share / "launch" / "record_camera.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("record_bag")),
        launch_arguments={
            "output": LaunchConfiguration("bag_output"),
            "camera_namespace": "sensors",
            "camera_name": "nav_camera",
            "stage": LaunchConfiguration("camera_stage"),
        }.items(),
    )
    perception = Node(
        package="target_perception",
        executable="instance_segmentation",
        name="instance_segmentation",
        output="screen",
        parameters=[
            str(perception_share / "config" / "instance_segmentation.yaml"),
            {
                "weights_path": LaunchConfiguration("weights_path"),
                "device": LaunchConfiguration("device"),
                "output_frame": LaunchConfiguration("output_frame"),
            },
        ],
    )
    tracking = Node(
        package="target_perception",
        executable="instance_tracking",
        name="instance_tracking",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_tracking")),
        parameters=[str(perception_share / "config" / "instance_tracking.yaml")],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="instance_perception_rviz",
        arguments=["-d", str(bringup_share / "rviz" / "instance_perception.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )
    benchmark = Node(
        package="target_perception",
        executable="general_object_benchmark",
        name="general_object_benchmark",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_benchmark")),
        parameters=[
            {
                "output_dir": LaunchConfiguration("benchmark_output_dir"),
                "sample_count": ParameterValue(
                    LaunchConfiguration("benchmark_samples"), value_type=int
                ),
                "warmup_frames": ParameterValue(
                    LaunchConfiguration("benchmark_warmup_frames"), value_type=int
                ),
                "expected_frame": LaunchConfiguration("output_frame"),
            }
        ],
        on_exit=Shutdown(reason="general-object benchmark finished"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera_stage", default_value="aligned"),
            DeclareLaunchArgument("reset_on_start", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_tracking", default_value="true"),
            DeclareLaunchArgument("run_benchmark", default_value="false"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument(
                "bag_output", default_value="bags/d435i_general_object_001"
            ),
            DeclareLaunchArgument("benchmark_samples", default_value="100"),
            DeclareLaunchArgument("benchmark_warmup_frames", default_value="15"),
            DeclareLaunchArgument(
                "benchmark_output_dir",
                default_value="/tmp/general_object_benchmark",
            ),
            DeclareLaunchArgument(
                "weights_path",
                default_value=str(
                    workspace
                    / "models"
                    / "maskrcnn_resnet50_fpn_v2_coco-73cbd019.pth"
                ),
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument(
                "output_frame", default_value="nav_camera_color_optical_frame"
            ),
            camera,
            recorder,
            perception,
            tracking,
            rviz,
            benchmark,
        ]
    )
