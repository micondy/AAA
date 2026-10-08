from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


STAGE_PARAMETERS = {
    # 逐级启用功能，便于定位 USB 带宽、固件或某一路数据流的问题。
    "basic": {
        "enable_color": True,
        "enable_depth": True,
        "enable_accel": False,
        "enable_gyro": False,
        "align_depth.enable": False,
        "pointcloud.enable": False,
    },
    "imu": {
        "enable_color": True,
        "enable_depth": True,
        "enable_accel": True,
        "enable_gyro": True,
        "align_depth.enable": False,
        "pointcloud.enable": False,
    },
    "aligned": {
        "enable_color": True,
        "enable_depth": True,
        "enable_accel": True,
        "enable_gyro": True,
        "align_depth.enable": True,
        "pointcloud.enable": False,
    },
    "full": {
        "enable_color": True,
        "enable_depth": True,
        "enable_accel": True,
        "enable_gyro": True,
        "align_depth.enable": True,
        "pointcloud.enable": True,
    },
}


def _load_ros_parameters(params_path: Path) -> dict:
    with params_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}

    # 支持标准 ROS 2 参数文件，也兼容直接写成平铺字典的配置文件。
    if "/**" in document:
        document = document["/**"]
    if "ros__parameters" in document:
        document = document["ros__parameters"]
    if not isinstance(document, dict):
        raise RuntimeError(f"参数文件内容不是字典: {params_path}")
    return document


def _create_camera_node(context):
    stage = LaunchConfiguration("stage").perform(context).strip().lower()
    if stage not in STAGE_PARAMETERS:
        choices = ", ".join(STAGE_PARAMETERS)
        raise RuntimeError(f"未知相机阶段 {stage!r}，可选值: {choices}")

    params_path = Path(LaunchConfiguration("params_file").perform(context)).expanduser()
    parameters = _load_ros_parameters(params_path)
    parameters.update(STAGE_PARAMETERS[stage])
    # realsense2_camera uses its `camera_name` parameter (not only the ROS node
    # name) to build frame IDs.  Keep topic and TF prefixes identical, e.g.
    # /sensors/nav_camera/... together with nav_camera_link.
    camera_name = LaunchConfiguration("camera_name").perform(context).strip("/")
    if not camera_name:
        raise RuntimeError("camera_name must not be empty")
    parameters["camera_name"] = camera_name
    parameters["use_sim_time"] = LaunchConfiguration("use_sim_time").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    reset_on_start = LaunchConfiguration("reset_on_start").perform(context).lower()
    parameters["initial_reset"] = reset_on_start in {
        "1", "true", "yes", "on"
    }

    serial_no = LaunchConfiguration("serial_no").perform(context).strip().strip("'\"")
    if serial_no:
        parameters["serial_no"] = serial_no

    return [Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=LaunchConfiguration("camera_namespace"),
        name=LaunchConfiguration("camera_name"),
        parameters=[parameters],
        output="screen",
        emulate_tty=True,
    )]


def _create_rviz(context):
    if LaunchConfiguration("use_rviz").perform(context).lower() not in {
        "1", "true", "yes", "on"
    }:
        return []

    namespace = LaunchConfiguration("camera_namespace").perform(context).strip("/")
    name = LaunchConfiguration("camera_name").perform(context).strip("/")
    if not namespace or not name:
        raise RuntimeError("camera_namespace and camera_name must not be empty")
    prefix = f"/{namespace}/{name}"
    stage = LaunchConfiguration("stage").perform(context).strip().lower()
    depth_topic = (
        f"{prefix}/aligned_depth_to_color/image_raw"
        if stage in ("aligned", "full")
        else f"{prefix}/depth/image_rect_raw"
    )
    bringup_share = Path(get_package_share_directory("d435i_bringup"))
    return [Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(bringup_share / "rviz" / "camera.rviz")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("/camera/camera/color/image_raw", f"{prefix}/color/image_raw"),
            ("/camera/camera/aligned_depth_to_color/image_raw", depth_topic),
            (
                "/camera/camera/depth/color/points",
                f"{prefix}/depth/color/points",
            ),
        ],
        output="screen",
    )]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("d435i_bringup"))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument(
            "serial_no", default_value="",
            description="RealSense serial; empty selects the first device",
        ),
        DeclareLaunchArgument(
            "stage", default_value="full",
            description="Camera feature stage: basic, imu, aligned, or full",
        ),
        DeclareLaunchArgument(
            "reset_on_start", default_value="false",
            description="Reset the RealSense once before opening streams",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=str(bringup_share / "config" / "realsense_d435i.yaml"),
        ),
        OpaqueFunction(function=_create_camera_node),
        OpaqueFunction(function=_create_rviz),
    ])
