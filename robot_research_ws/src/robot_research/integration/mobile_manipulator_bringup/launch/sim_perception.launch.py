"""Thin Isaac Sim profile for RGB-D perception and the unified camera TF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _distance_values(context):
    raw = LaunchConfiguration("benchmark_distances").perform(context)
    try:
        values = tuple(float(item.strip()) for item in raw.split(",") if item.strip())
    except ValueError as error:
        raise RuntimeError("benchmark_distances must be comma-separated numbers") from error
    if not values or any(not 0.2 <= value <= 3.0 for value in values):
        raise RuntimeError("benchmark_distances must contain values in [0.2, 3.0]")
    return values


def _isaac_process(context):
    if not _enabled(context, "start_isaac"):
        return []

    python_executable = Path(LaunchConfiguration("isaac_python").perform(context))
    script = Path(LaunchConfiguration("isaac_script").perform(context))
    if not python_executable.is_file():
        raise RuntimeError(f"Isaac Sim Python executable does not exist: {python_executable}")
    if not script.is_file():
        raise RuntimeError(f"Isaac Sim runner does not exist: {script}")

    if _enabled(context, "run_benchmark") and _enabled(
        context, "run_distance_benchmark"
    ):
        raise RuntimeError("run_benchmark and run_distance_benchmark are mutually exclusive")
    if (
        _enabled(context, "run_benchmark")
        or _enabled(context, "run_distance_benchmark")
    ) and (
        not _enabled(context, "add_green_target")
        or not _enabled(context, "start_target_perception")
    ):
        raise RuntimeError("benchmarks require add_green_target and target_perception")

    command = [str(python_executable), str(script), "--enable-nav-rgbd"]
    if _enabled(context, "empty_scene"):
        command.append("--empty-scene")
    if _enabled(context, "enable_base_control"):
        command.append("--ros")
    if _enabled(context, "add_rack"):
        command.append("--add-rack")
        command.extend(("--rack-layout", LaunchConfiguration("rack_layout").perform(context)))
        command.extend(("--rack-visual-seed", LaunchConfiguration("rack_visual_seed").perform(context)))
        rack_asset = LaunchConfiguration("rack_asset").perform(context).strip()
        if rack_asset:
            command.extend(("--rack-asset", rack_asset))
        if _enabled(context, "parametric_rack"):
            command.append("--parametric-rack")
    if _enabled(context, "add_green_target"):
        command.extend(
            (
                "--add-green-target",
                "--green-target-depth",
                LaunchConfiguration("green_target_depth").perform(context),
            )
        )
        if _enabled(context, "run_distance_benchmark"):
            command.append("--green-target-depth-sequence")
            command.extend(str(value) for value in _distance_values(context))
            command.extend(
                (
                    "--green-target-stage-seconds",
                    LaunchConfiguration("benchmark_stage_seconds").perform(context),
                )
            )
    if _enabled(context, "headless"):
        command.append("--headless")
    test_seconds = LaunchConfiguration("test_seconds").perform(context).strip()
    if test_seconds not in ("", "0", "0.0"):
        command.extend(("--test-seconds", test_seconds))
    elif _enabled(context, "run_distance_benchmark"):
        stage_seconds = float(
            LaunchConfiguration("benchmark_stage_seconds").perform(context)
        )
        duration = len(_distance_values(context)) * stage_seconds + 1.0
        command.extend(("--test-seconds", str(duration)))
    elif _enabled(context, "run_benchmark"):
        samples = int(LaunchConfiguration("benchmark_samples").perform(context))
        warmup = int(LaunchConfiguration("benchmark_warmup_frames").perform(context))
        duration = max(30.0, (samples + warmup) / 15.0 + 2.0)
        command.extend(("--test-seconds", str(duration)))

    return [
        ExecuteProcess(
            cmd=command,
            output="screen",
            additional_env={"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"},
            on_exit=Shutdown(reason="Isaac Sim process exited"),
        )
    ]


def _benchmark_node(context):
    if not _enabled(context, "run_benchmark"):
        return []
    depth = float(LaunchConfiguration("green_target_depth").perform(context))
    truth_x = 0.3306 + depth
    return [
        Node(
            package="target_perception",
            executable="benchmark_recorder",
            name="target_perception_benchmark",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "output_csv": ParameterValue(
                        LaunchConfiguration("benchmark_output_csv"), value_type=str
                    ),
                    "sample_count": ParameterValue(
                        LaunchConfiguration("benchmark_samples"), value_type=int
                    ),
                    "warmup_frames": ParameterValue(
                        LaunchConfiguration("benchmark_warmup_frames"), value_type=int
                    ),
                    "expected_frame": "base_link",
                    "truth_front_depth_m": depth,
                    "truth_x_m": truth_x,
                    "truth_y_m": 0.0,
                    "truth_z_m": 0.4325,
                    "truth_size_x_m": 0.20,
                    "truth_size_y_m": 0.20,
                }
            ],
        )
    ]


def _distance_benchmark_node(context):
    if not _enabled(context, "run_distance_benchmark"):
        return []
    distances = _distance_values(context)
    stage_seconds = float(
        LaunchConfiguration("benchmark_stage_seconds").perform(context)
    )
    samples = int(LaunchConfiguration("benchmark_samples").perform(context))
    warmup = int(LaunchConfiguration("benchmark_warmup_frames").perform(context))
    minimum_stage_seconds = (samples + warmup) / 15.0 + 1.0
    if stage_seconds < minimum_stage_seconds:
        raise RuntimeError(
            "benchmark_stage_seconds is too short for the requested frame count; "
            f"use at least {minimum_stage_seconds:.2f} s"
        )
    return [
        Node(
            package="target_perception",
            executable="distance_benchmark_recorder",
            name="target_perception_distance_benchmark",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "output_dir": ParameterValue(
                        LaunchConfiguration("benchmark_output_dir"), value_type=str
                    ),
                    "distances_m": list(distances),
                    "stage_duration_s": ParameterValue(
                        LaunchConfiguration("benchmark_stage_seconds"), value_type=float
                    ),
                    "samples_per_distance": ParameterValue(
                        LaunchConfiguration("benchmark_samples"), value_type=int
                    ),
                    "warmup_frames_per_distance": ParameterValue(
                        LaunchConfiguration("benchmark_warmup_frames"), value_type=int
                    ),
                }
            ],
        )
    ]


def generate_launch_description():
    description_share = Path(
        get_package_share_directory("mobile_manipulator_description")
    )
    perception_share = Path(get_package_share_directory("target_perception"))
    bringup_share = Path(get_package_share_directory("mobile_manipulator_bringup"))
    workspace = bringup_share.parents[3]

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            str(description_share / "urdf" / "mobile_manipulator.urdf.xacro"),
            " use_mock_hardware:=true",
            " openarm_ros2_control:=false",
            " nav_camera_enabled:=true",
            " nav_camera_use_nominal_extrinsics:=true",
            " nav_camera_mount_xyz:='",
            LaunchConfiguration("nav_camera_mount_xyz"),
            "' nav_camera_mount_rpy:='",
            LaunchConfiguration("nav_camera_mount_rpy"),
            "'",
        ]
    )

    description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": True,
            }
        ],
    )
    target_perception = Node(
        package="target_perception",
        executable="color_depth_detector",
        name="color_depth_detector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_target_perception")),
        parameters=[
            str(perception_share / "config" / "color_depth.yaml"),
            {
                "output_frame": LaunchConfiguration("output_frame"),
                "use_sim_time": True,
            },
        ],
    )
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_world_to_base_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_world_tf")),
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_isaac", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("empty_scene", default_value="false"),
            DeclareLaunchArgument("test_seconds", default_value="0"),
            DeclareLaunchArgument("add_green_target", default_value="true"),
            DeclareLaunchArgument("add_rack", default_value="false"),
            DeclareLaunchArgument("rack_asset", default_value=""),
            DeclareLaunchArgument("rack_layout", default_value="grid4x4"),
            DeclareLaunchArgument("rack_visual_seed", default_value="0"),
            DeclareLaunchArgument("parametric_rack", default_value="false"),
            DeclareLaunchArgument("green_target_depth", default_value="0.77"),
            DeclareLaunchArgument("start_target_perception", default_value="true"),
            DeclareLaunchArgument("enable_base_control", default_value="false"),
            DeclareLaunchArgument("output_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_world_tf", default_value="false"),
            DeclareLaunchArgument("run_benchmark", default_value="false"),
            DeclareLaunchArgument("run_distance_benchmark", default_value="false"),
            DeclareLaunchArgument(
                "benchmark_distances", default_value="0.4,0.6,0.8,1.0"
            ),
            DeclareLaunchArgument("benchmark_stage_seconds", default_value="30.0"),
            DeclareLaunchArgument("benchmark_samples", default_value="100"),
            DeclareLaunchArgument("benchmark_warmup_frames", default_value="15"),
            DeclareLaunchArgument(
                "benchmark_output_csv",
                default_value="/tmp/target_perception_benchmark.csv",
            ),
            DeclareLaunchArgument(
                "benchmark_output_dir",
                default_value="/tmp/target_perception_distance_benchmark",
            ),
            DeclareLaunchArgument("nav_camera_mount_xyz", default_value="0.10 0 1.70"),
            DeclareLaunchArgument("nav_camera_mount_rpy", default_value="0 0.20944 0"),
            DeclareLaunchArgument(
                "isaac_python",
                default_value="/home/scc/isaac-sim-standalone-6.0.1-linux-x86_64/python.sh",
            ),
            DeclareLaunchArgument(
                "isaac_script",
                default_value=str(workspace / "tools" / "isaac_sim" / "run_mobile_manipulator.py"),
            ),
            description,
            target_perception,
            world_tf,
            OpaqueFunction(function=_benchmark_node),
            OpaqueFunction(function=_distance_benchmark_node),
            OpaqueFunction(function=_isaac_process),
        ]
    )
