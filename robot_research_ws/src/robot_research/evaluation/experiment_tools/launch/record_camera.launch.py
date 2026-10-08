import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from experiment_tools.camera_topics import camera_topics


def _qos_text(topics: list[str]) -> str:
    blocks = []
    for topic in topics:
        reliability = "reliable" if (
            topic.endswith("camera_info")
            or topic in ("/diagnostics", "/tf", "/tf_static")
        ) else "best_effort"
        durability = "transient_local" if topic == "/tf_static" else "volatile"
        depth = 100 if topic.endswith(("/imu", "/sample")) else 10
        blocks.append(
            f"{topic}:\n"
            f"  reliability: {reliability}\n"
            f"  durability: {durability}\n"
            "  history: keep_last\n"
            f"  depth: {depth}\n"
        )
    return "".join(blocks)


def _create_recorder(context):
    topics = camera_topics(
        LaunchConfiguration("camera_namespace").perform(context),
        LaunchConfiguration("camera_name").perform(context),
        LaunchConfiguration("stage").perform(context),
    )
    qos_file = LaunchConfiguration("qos_file").perform(context).strip()
    if not qos_file:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", suffix=".yaml", delete=False
        ) as stream:
            stream.write(_qos_text(topics))
            qos_file = stream.name

    return [ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--output", LaunchConfiguration("output"),
            "--qos-profile-overrides-path", qos_file,
            *topics,
        ],
        output="screen",
        sigterm_timeout="10",
        sigkill_timeout="5",
    )]


def generate_launch_description():
    share = Path(get_package_share_directory("experiment_tools"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "output", default_value="camera_stage_a",
            description="Output rosbag directory; choose a new name for every run",
        ),
        DeclareLaunchArgument(
            "qos_file", default_value="",
            description=(
                "Optional QoS override YAML. Empty generates one for the selected namespace. "
                "The default-topic template is installed at "
                f"{share / 'config' / 'rosbag_qos.yaml'}"
            ),
        ),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument(
            "stage", default_value="full",
            description="Must match the stage used by d435i_bringup",
        ),
        OpaqueFunction(function=_create_recorder),
    ])
