from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory("omni_lattice_benchmark"))
    map_file = LaunchConfiguration("map")
    planner_profile = LaunchConfiguration("planner_profile")
    lattice_file = LaunchConfiguration("lattice_file")

    configured_planner = RewrittenYaml(
        source_file=planner_profile,
        param_rewrites={"lattice_filepath": lattice_file},
        convert_types=True,
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[str(share / "config" / "common.yaml"), {"yaml_filename": map_file}],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[str(share / "config" / "common.yaml"), configured_planner],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planner_benchmark",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server", "planner_server"]}],
    )
    fixed_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="benchmark_map_to_base",
        arguments=[
            "--x", "1.0", "--y", "1.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "map", "--child-frame-id", "base_link",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=str(share / "maps" / "open_space.yaml")),
            DeclareLaunchArgument(
                "planner_profile",
                default_value=str(share / "config" / "planner_navfn.yaml"),
            ),
            DeclareLaunchArgument(
                "lattice_file",
                default_value=str(share / "lattice" / "nav2_official_omni_16_5cm_r0.5.json"),
            ),
            fixed_tf,
            map_server,
            planner_server,
            lifecycle_manager,
        ]
    )
