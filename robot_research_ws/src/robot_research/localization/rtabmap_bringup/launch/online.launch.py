from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rtabmap_bringup.configuration import (
    as_bool,
    camera_prefix,
    load_flat_yaml,
    mode_parameters,
)


def _create_rtabmap_nodes(context):
    share = Path(get_package_share_directory("rtabmap_bringup"))
    namespace = LaunchConfiguration("camera_namespace").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    camera = camera_prefix(namespace, camera_name)
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    odom_source = LaunchConfiguration("odom_source").perform(context).strip().lower()
    if odom_source not in {"rgbd", "external"}:
        raise ValueError("odom_source must be 'rgbd' or 'external'")
    odom_topic = LaunchConfiguration("odom_topic").perform(context).strip()
    if odom_source == "external" and not odom_topic.startswith("/"):
        raise ValueError("external odom_topic must be an absolute ROS topic")
    database = Path(
        LaunchConfiguration("database_path").perform(context)
    ).expanduser()

    if mode == "localization" and not database.is_file():
        raise RuntimeError(f"RTAB-Map database does not exist: {database}")

    odom_parameters = load_flat_yaml(
        LaunchConfiguration("odom_params_file").perform(context)
    )
    slam_parameters = load_flat_yaml(
        LaunchConfiguration("slam_params_file").perform(context)
    )
    odom_parameters.update({
        "frame_id": LaunchConfiguration("base_frame").perform(context),
        "odom_frame_id": LaunchConfiguration("odom_frame").perform(context),
        "publish_tf": as_bool(LaunchConfiguration("publish_odom_tf").perform(context)),
        "use_sim_time": as_bool(LaunchConfiguration("use_sim_time").perform(context)),
    })
    slam_parameters.update(mode_parameters(mode))
    slam_parameters.update({
        "database_path": str(database),
        "frame_id": LaunchConfiguration("base_frame").perform(context),
        "map_frame_id": LaunchConfiguration("map_frame").perform(context),
        "publish_tf": as_bool(LaunchConfiguration("publish_map_tf").perform(context)),
        "use_sim_time": as_bool(LaunchConfiguration("use_sim_time").perform(context)),
    })
    if odom_source == "external":
        # Wheel/chassis odometry owns odom -> base_link.  RTAB-Map consumes
        # that motion estimate and only owns the map -> odom correction.
        slam_parameters["subscribe_odom_info"] = False
        slam_parameters["Mem/UseOdomFeatures"] = "false"
    initial_pose = LaunchConfiguration("initial_pose").perform(context).strip()
    if initial_pose:
        slam_parameters["initial_pose"] = initial_pose

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        namespace="rtabmap",
        name="rgbd_sync",
        parameters=[{
            "approx_sync": False,
            "qos": 2,
            "qos_camera_info": 2,
        }],
        remappings=[
            ("rgb/image", f"{camera}/color/image_raw"),
            ("depth/image", f"{camera}/aligned_depth_to_color/image_raw"),
            ("rgb/camera_info", f"{camera}/color/camera_info"),
            ("rgbd_image", "/rtabmap/rgbd_image"),
        ],
        output="screen",
    )
    arguments = []
    if mode == "mapping" and as_bool(
        LaunchConfiguration("fresh_map").perform(context)
    ):
        arguments.append("--delete_db_on_start")
    slam_remappings = [
        (
            "rgbd_image",
            "/rtabmap/odom_rgbd_image"
            if odom_source == "rgbd"
            else "/rtabmap/rgbd_image",
        ),
        ("odom", "/rtabmap/odom" if odom_source == "rgbd" else odom_topic),
    ]
    if odom_source == "rgbd":
        slam_remappings.append(("odom_info", "/rtabmap/odom_info"))
    slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        namespace="rtabmap",
        name="rtabmap",
        parameters=[slam_parameters],
        remappings=slam_remappings,
        arguments=arguments,
        output="screen",
    )
    nodes = [rgbd_sync]
    if odom_source == "rgbd":
        nodes.append(Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            namespace="rtabmap",
            name="rgbd_odometry",
            parameters=[odom_parameters],
            remappings=[
                ("rgbd_image", "/rtabmap/rgbd_image"),
                ("odom_rgbd_image", "/rtabmap/odom_rgbd_image"),
                ("odom", "/rtabmap/odom"),
                ("odom_info", "/rtabmap/odom_info"),
            ],
            output="screen",
        ))
    nodes.append(slam)
    # This function already runs with a resolved launch context. Evaluating the
    # flag here avoids a deferred condition being lost when this launch file is
    # nested through another IncludeLaunchDescription.
    if as_bool(LaunchConfiguration("use_rviz").perform(context)):
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "rtabmap.rviz")],
            parameters=[{
                "use_sim_time": as_bool(
                    LaunchConfiguration("use_sim_time").perform(context)
                )
            }],
            output="screen",
        ))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("rtabmap_bringup"))
    camera_share = Path(get_package_share_directory("d435i_bringup"))
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(camera_share / "launch" / "camera.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_camera")),
        launch_arguments={
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "camera_name": LaunchConfiguration("camera_name"),
            "serial_no": LaunchConfiguration("serial_no"),
            "stage": LaunchConfiguration("camera_stage"),
            "use_rviz": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument(
            "serial_no", default_value="",
            description="D435i serial; set this when multiple RealSense devices are connected",
        ),
        DeclareLaunchArgument(
            "camera_stage", default_value="aligned",
            description="D435i stage; use full when Nav2 consumes the point cloud",
        ),
        DeclareLaunchArgument("base_frame", default_value="camera_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument(
            "odom_source",
            default_value="rgbd",
            description="Continuous odometry source: rgbd or external",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/swerve_controller/odom",
            description="nav_msgs/Odometry input used when odom_source:=external",
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument(
            "initial_pose",
            default_value="",
            description="Optional RTAB-Map startup pose: x y z roll pitch yaw",
        ),
        DeclareLaunchArgument("publish_odom_tf", default_value="true"),
        DeclareLaunchArgument("publish_map_tf", default_value="true"),
        DeclareLaunchArgument(
            "mode", default_value="mapping",
            description="RTAB-Map mode: mapping or localization",
        ),
        DeclareLaunchArgument(
            "fresh_map", default_value="false",
            description="Delete the selected database before mapping starts",
        ),
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/rtabmap.db"
        ),
        DeclareLaunchArgument(
            "odom_params_file",
            default_value=str(share / "config" / "rgbd_odometry.yaml"),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=str(share / "config" / "rtabmap.yaml"),
        ),
        OpaqueFunction(function=_create_rtabmap_nodes),
        # The camera include sets its own use_rviz:=false. Create the SLAM
        # nodes first so the outer use_rviz value is evaluated before entering
        # the included launch scope.
        camera,
    ])
