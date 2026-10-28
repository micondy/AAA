import ast
from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_nav2():
    with (PACKAGE_ROOT / "config" / "nav2.yaml").open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_costmap_overlay(name):
    with (PACKAGE_ROOT / "config" / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_controller_uses_mppi_omni_with_lateral_motion_enabled():
    config = load_nav2()
    controller = config["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]

    smoother = config["velocity_smoother"]["ros__parameters"]

    assert follow["plugin"].endswith("MPPIController")
    assert follow["motion_model"] == "Omni"
    assert 0.0 < follow["vy_max"] <= follow["vx_max"]
    assert "PreferForwardCritic" not in follow["critics"]
    assert follow["PathAngleCritic"]["mode"] == 2
    assert follow["CostCritic"]["consider_footprint"] is True
    assert smoother["max_velocity"][1] > 0.0
    assert smoother["min_velocity"][1] < 0.0
    assert controller["progress_checker"]["plugin"].endswith("PoseProgressChecker")
    assert controller["progress_checker"]["required_movement_angle"] > 0.0


def test_global_planner_uses_omni_state_lattice():
    config = load_nav2()
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    lattice = PACKAGE_ROOT / "lattice" / "nav2_official_omni_16_5cm_r0.5.json"
    launch_text = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert planner["plugin"].endswith("SmacPlannerLattice")
    assert planner["lattice_filepath"] == "__LATTICE_FILE__"
    assert lattice.is_file()
    assert '"lattice_filepath": LaunchConfiguration("lattice_file")' in launch_text
    assert "nav2_official_omni_16_5cm_r0.5.json" in launch_text
    assert 'get_package_share_directory("omni_lattice_benchmark")' not in launch_text


def test_path_smoother_is_configured_and_lifecycle_managed():
    config = load_nav2()
    smoother = config["smoother_server"]["ros__parameters"]
    launch_text = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert smoother["smoother_plugins"] == ["simple_smoother"]
    assert smoother["simple_smoother"]["plugin"] == "nav2_smoother::SimpleSmoother"
    assert 'package="nav2_smoother"' in launch_text
    assert '"smoother_server",' in launch_text


def test_costmaps_use_rtabmap_and_fixed_front_camera():
    config = load_nav2()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]

    expected_cloud = "/sensors/nav_camera/depth/color/points"
    assert local["voxel_layer"]["front_depth"]["topic"] == expected_cloud
    assert global_map["obstacle_layer"]["front_depth"]["topic"] == expected_cloud
    assert global_map["static_layer"]["map_topic"] == "/rtabmap/map"
    assert local["robot_base_frame"] == "base_link"
    assert local["global_frame"] == "odom"
    assert global_map["global_frame"] == "map"
    # Nav2 Jazzy declares these two costmap parameters as integers.
    assert type(local["width"]) is int
    assert type(local["height"]) is int
    assert local["voxel_layer"]["z_voxels"] <= 16
    assert (
        local["voxel_layer"]["z_resolution"]
        * local["voxel_layer"]["z_voxels"]
        >= local["voxel_layer"]["max_obstacle_height"]
    )
    assert local["inflation_layer"]["inflation_radius"] >= 0.59
    assert global_map["inflation_layer"]["inflation_radius"] >= 0.59


def test_hardware_profile_has_four_serial_bound_cameras_and_two_lidars():
    with (PACKAGE_ROOT / "config" / "hardware_sensors.yaml").open(
        encoding="utf-8"
    ) as stream:
        sensors = yaml.safe_load(stream)
    overlay = load_costmap_overlay("costmap_multi_sensor_overlay.yaml")
    launch_text = (
        PACKAGE_ROOT / "launch" / "hardware_multi_sensor_navigation.launch.py"
    ).read_text(encoding="utf-8")

    assert [camera["name"] for camera in sensors["cameras"]] == [
        "nav_camera", "rear_camera", "left_camera", "right_camera"
    ]
    assert len({camera["serial_no"] for camera in sensors["cameras"]}) == 4
    assert [lidar["name"] for lidar in sensors["lidars"]] == [
        "front_lidar", "rear_lidar"
    ]
    assert '"start_camera": "false"' in launch_text
    assert '"odom_source": "external"' in launch_text
    assert 'camera_namespace": "sensors"' in launch_text
    assert 'Set the measured serial_no' in launch_text

    expected_sources = {
        "front_depth", "rear_depth", "left_depth", "right_depth",
        "front_laser", "rear_laser",
    }
    local = overlay["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]
    global_layer = overlay["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    monitor = overlay["collision_monitor"]["ros__parameters"]
    assert set(local["observation_sources"].split()) == expected_sources
    assert set(global_layer["observation_sources"].split()) == expected_sources
    assert set(monitor["observation_sources"]) == expected_sources
    assert local["front_laser"]["data_type"] == "LaserScan"
    assert monitor["front_laser"]["type"] == "scan"


def test_isaac_multi_sensor_profile_matches_hardware_topics():
    simulator = (
        PACKAGE_ROOT.parents[3] / "tools" / "isaac_sim" / "run_mobile_manipulator.py"
    ).read_text(encoding="utf-8")
    launch_text = (
        PACKAGE_ROOT / "launch" / "mobile_base_multi_sensor_sim_navigation.launch.py"
    ).read_text(encoding="utf-8")

    assert '"--enable-multi-sensor-nav"' in simulator
    for camera in ("nav_camera", "rear_camera", "left_camera", "right_camera"):
        assert f'(\"{camera}\",' in simulator
    for lidar in ("front_lidar", "rear_lidar"):
        assert f'(\"{lidar}\",' in simulator
    assert 'config="Example_Rotary_2D"' in simulator
    assert 'f"sensors/{name}/scan"' in simulator
    assert '"costmap_multi_sensor_overlay.yaml"' in launch_text
    assert '"start_camera": "false"' in launch_text
    assert '"use_sim_time": "true"' in launch_text
    assert '"collision_source_timeout": "2.0"' in launch_text


def test_carter_navigation_profile_pins_sim_clock_and_safe_output():
    launch_text = (PACKAGE_ROOT / "launch" / "carter_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"use_sim_time": "true"' in launch_text
    assert '"start_camera": "false"' in launch_text
    assert '"cmd_vel_out_topic": "/cmd_vel"' in launch_text
    assert '"front_stereo_camera_left_rgb"' in launch_text
    assert 'executable="static_transform_publisher"' in launch_text


def test_custom_mobile_base_profiles_use_unified_description_and_interfaces():
    mapping = (
        PACKAGE_ROOT / "launch" / "mobile_base_sim_mapping.launch.py"
    ).read_text(encoding="utf-8")
    navigation = (
        PACKAGE_ROOT / "launch" / "mobile_base_sim_navigation.launch.py"
    ).read_text(encoding="utf-8")

    for source in (mapping, navigation):
        assert '"use_sim_time": "true"' in source
        assert '"start_camera": "false"' in source
        assert '"nav_camera_use_nominal_extrinsics": "true"' in source
        assert '"use_joint_state_publisher": "false"' in source
        assert '"display.launch.py"' in source
        assert '"odom_source": "external"' in source
        assert '"odom_topic": "/swerve_controller/odom"' in source
        assert 'DeclareLaunchArgument("enable_nvblox", default_value="false")' in source
        assert '"input_qos": "DEFAULT"' in source
    assert 'condition=IfCondition(LaunchConfiguration("enable_nvblox"))' in mapping
    assert 'PythonExpression([' in navigation
    assert '"cmd_vel_out_topic": "/cmd_vel_nav_safe"' in navigation
    assert '"cmd_vel_smoothed_topic": "/cmd_vel_smoothed"' in navigation
    assert '"collision_source_timeout": "2.0"' in navigation
    assert '"xy_goal_tolerance": "0.08"' in navigation
    assert '"yaw_goal_tolerance": "0.15"' in navigation
    assert '"required_movement_radius": "0.08"' in navigation
    assert 'executable="rviz2"' in mapping
    assert 'executable="rviz2"' in navigation
    assert 'OpaqueFunction(function=launch_rviz)' in navigation
    assert 'LaunchConfiguration("use_rviz").perform(context)' in navigation
    assert 'OpaqueFunction(function=launch_rviz)' in mapping
    assert 'LaunchConfiguration("use_rviz").perform(context)' in mapping
    assert '"initial_pose": LaunchConfiguration("initial_pose")' in navigation
    assert 'default_value="0 0 0 0 0 0"' in navigation
    assert '"use_rviz": "false"' in mapping
    assert '"use_rviz": "false"' in navigation
    assert 'DeclareLaunchArgument(\n            "start_web_console"' in navigation
    assert 'package="mobile_base_web_console"' in navigation
    assert '("/swerve_controller/cmd_vel", "/cmd_vel_web")' in navigation
    assert 'executable="cmd_vel_priority_mux.py"' in navigation
    assert '("/map", "/rtabmap/map")' in navigation
    assert '"MOBILE_BASE_SIMULATION_DIRECT_CONTROL": "true"' in navigation
    assert '"rtabmap_mode": LaunchConfiguration("rtabmap_mode")' in navigation


def test_navigation_supports_online_mapping_with_nav2():
    launch_text = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"online.launch.py"' in launch_text
    assert '"mode": LaunchConfiguration("rtabmap_mode")' in launch_text
    assert '"fresh_map": LaunchConfiguration("fresh_map")' in launch_text


def test_nvblox_costmap_profile_is_optional_and_preserves_global_static_map():
    overlay = load_costmap_overlay("costmap_nvblox_overlay.yaml")
    local = overlay["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = overlay["global_costmap"]["global_costmap"]["ros__parameters"]
    launch_text = (
        PACKAGE_ROOT / "launch" / "mobile_base_sim_navigation.launch.py"
    ).read_text(encoding="utf-8")
    generic_launch = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert local["plugins"] == ["nvblox_layer", "inflation_layer"]
    assert global_map["plugins"] == [
        "static_layer",
        "nvblox_layer",
        "inflation_layer",
    ]
    for costmap, frame in ((local, "odom"), (global_map, "map")):
        layer = costmap["nvblox_layer"]
        assert layer["plugin"] == "nvblox::nav2::NvbloxCostmapLayer"
        assert layer["nvblox_map_slice_topic"] == "/nvblox_node/static_map_slice"
        assert layer["nav2_costmap_global_frame"] == frame
        assert layer["convert_to_binary_costmap"] is True

    assert 'DeclareLaunchArgument(\n            "use_nvblox_costmap"' in launch_text
    assert 'default_value="false"' in launch_text
    assert 'UnlessCondition(use_nvblox_costmap)' in launch_text
    assert 'IfCondition(use_nvblox_costmap)' in launch_text
    assert '"costmap_nvblox_overlay.yaml"' in launch_text
    assert '"costmap_baseline_overlay.yaml"' in launch_text
    assert 'DeclareLaunchArgument(\n            "costmap_overlay_file"' in generic_launch
    assert '"parameters": [configured_params, costmap_overlay]' in generic_launch


def test_isaac_mobile_manipulator_separates_controller_odom_from_truth():
    simulator = (
        PACKAGE_ROOT.parents[3] / "tools" / "isaac_sim" / "run_mobile_manipulator.py"
    ).read_text(encoding="utf-8")

    assert '"ControllerOdomPub"' in simulator
    assert '"swerve_controller/odom"' in simulator
    assert '"OdomTfPub"' in simulator
    assert '"OdomTfPub.inputs:parentFrameId", "odom"' in simulator
    assert '"GroundTruthOdomPub"' in simulator
    assert '"sim/ground_truth/odom"' in simulator
    assert '"GroundTruthTfPub.inputs:topicName", "sim/ground_truth/tf"' in simulator
    assert "received_motion_command" in simulator
    assert "MIN_SIM_PURE_ROTATION_SPEED = 0.12" in simulator
    assert "math.copysign(MIN_SIM_PURE_ROTATION_SPEED, wz)" in simulator
    assert "def steering_aligned" in simulator
    assert "if ros_input.steering_aligned(held_steering)" in simulator


def test_real_navigation_keeps_strict_collision_sensor_timeout():
    navigation = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"collision_source_timeout"' in navigation
    assert 'default_value="0.75"' in navigation
    assert '"source_timeout": LaunchConfiguration("collision_source_timeout")' in navigation


def test_velocity_chain_ends_at_swerve_controller():
    config = load_nav2()
    monitor = config["collision_monitor"]["ros__parameters"]

    assert monitor["cmd_vel_in_topic"] == "/cmd_vel_smoothed"
    assert monitor["cmd_vel_out_topic"] == "/swerve_controller/cmd_vel"
    assert monitor["observation_sources"] == ["front_depth"]
    assert monitor["front_depth"]["use_global_height"] is False
    assert monitor["base_shift_correction"] is False
    assert monitor["source_timeout"] <= 0.75
    stop_points = ast.literal_eval(monitor["FrontStop"]["points"])
    assert max(point[0] for point in stop_points) >= 1.0


def test_carter_route_is_installed_and_has_stop_watchdog():
    route = PACKAGE_ROOT / "scripts" / "carter_mapping_route.py"
    source = route.read_text(encoding="utf-8")

    assert route.exists()
    assert '"/sim/ground_truth/odom"' in source
    assert '"/rtabmap/odom"' in source
    assert '"/cmd_vel"' in source
    assert '"--estimate-topic"' in source
    assert "args.estimate_topic" in source
    assert "segment-timeout" in source
    assert "publish_stop" in source


def test_safe_behavior_tree_has_no_blind_reverse_recovery():
    tree = ElementTree.parse(PACKAGE_ROOT / "config" / "navigate_to_pose_safe.xml")
    tags = {element.tag for element in tree.iter()}

    assert "BackUp" not in tags
    assert "Spin" in tags
    assert "Wait" in tags


def test_waypoint_benchmark_is_explicit_and_repeatable():
    with (PACKAGE_ROOT / "config" / "isaac_indoor_top_rt_01_waypoints.yaml").open(
        encoding="utf-8"
    ) as stream:
        config = yaml.safe_load(stream)
    source = (PACKAGE_ROOT / "scripts" / "navigation_waypoint_benchmark.py").read_text(
        encoding="utf-8"
    )

    assert config["frame_id"] == "map"
    assert len(config["waypoints"]) == 10
    assert config["selection"]["minimum_map_clearance_m"] >= 0.65
    assert '"--execute"' in source
    assert '"--start-index"' in source
    assert '"--end-index"' in source
    assert "Selected {len(selected_waypoints)}" in source
    assert "cancel_goal_async" in source


def test_compact_external_odom_waypoints_and_variable_count_are_supported():
    with (
        PACKAGE_ROOT / "config" / "isaac_indoor_external_odom_04_waypoints.yaml"
    ).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    source = (PACKAGE_ROOT / "scripts" / "navigation_waypoint_benchmark.py").read_text(
        encoding="utf-8"
    )

    assert config["map_database"].endswith("isaac_indoor_external_odom_04.db")
    assert len(config["waypoints"]) == 4
    assert 'if not waypoints:' in source
    assert 'end_index = len(waypoints)' in source


def test_collision_safety_benchmark_covers_stop_and_sensor_timeout():
    source = (PACKAGE_ROOT / "scripts" / "collision_safety_benchmark.py").read_text(
        encoding="utf-8"
    )
    launch = (
        PACKAGE_ROOT / "launch" / "collision_safety_benchmark.launch.py"
    ).read_text(encoding="utf-8")

    assert "clear_cloud_allows_motion" in source
    assert "front_stop_blocks_motion" in source
    assert "pointcloud_timeout_blocks_motion" in source
    assert 'header.frame_id = "base_link"' in source
    assert '"front_depth.topic": "/safety_test/points"' in launch
    assert '"bond_timeout": 0.0' in launch


def test_isaac_obstacle_benchmark_uses_rendered_pointcloud_contract():
    source = (
        PACKAGE_ROOT / "scripts" / "isaac_obstacle_safety_benchmark.py"
    ).read_text(encoding="utf-8")
    launch = (
        PACKAGE_ROOT / "launch" / "isaac_obstacle_safety_benchmark.launch.py"
    ).read_text(encoding="utf-8")

    for obstacle_type in ("low", "hanging", "shelf"):
        assert obstacle_type in source
    assert 'f"/sim/nav_obstacle/{obstacle_type}/set_enabled"' in source
    assert 'f"{obstacle_type}_rendered_obstacle"' in source
    assert "cleared_after_removal" in source
    assert '"source_timeout": 2.0' in launch
    assert '"use_joint_state_publisher": "false"' in launch
    assert '"front_depth.topic"' not in launch


def test_isaac_scene_exposes_typed_navigation_obstacles():
    simulator = (
        PACKAGE_ROOT.parents[3] / "tools" / "isaac_sim" / "run_mobile_manipulator.py"
    ).read_text(encoding="utf-8")

    assert '"low": "/World/NavTestObstacleLow"' in simulator
    assert '"hanging": "/World/NavTestObstacleHanging"' in simulator
    assert '"shelf": "/World/NavTestObstacleShelf"' in simulator
    assert 'f"/sim/nav_obstacle/{obstacle_type}/set_enabled"' in simulator
    assert "Only one controlled obstacle is active at a time" in simulator


def test_nvblox_costmap_benchmark_checks_mark_and_clear_per_obstacle():
    source = (
        PACKAGE_ROOT / "scripts" / "nvblox_costmap_obstacle_benchmark.py"
    ).read_text(encoding="utf-8")

    assert '"/local_costmap/costmap"' in source
    assert '"/swerve_controller/odom"' in source
    assert "lethal_cells_in_front" in source
    assert '"mark_passed"' in source
    assert '"clear_passed"' in source
    for obstacle_type in ("low", "hanging", "shelf"):
        assert obstacle_type in source


def test_dynamic_intrusion_benchmark_measures_motion_clearance_and_dropout():
    source = (
        PACKAGE_ROOT / "scripts" / "dynamic_obstacle_intrusion_benchmark.py"
    ).read_text(encoding="utf-8")
    launch = (
        PACKAGE_ROOT / "launch" / "dynamic_obstacle_intrusion_benchmark.launch.py"
    ).read_text(encoding="utf-8")

    assert 'node.run_intrusion("low", 0.10, args)' in source
    assert 'node.run_intrusion("low", 0.18, args)' in source
    assert 'node.run_intrusion("hanging", 0.18, args)' in source
    assert "estimated_final_clearance_m" in source
    assert "moving_before_intrusion" in source
    assert "run_dropout" in source
    assert 'self.declare_parameter("allow_motion", False)' in source
    assert 'LaunchConfiguration("allow_motion"), value_type=bool' in launch
    assert 'default_value="false"' in launch
    assert '"cmd_vel_out_topic": "/cmd_vel"' in launch
    assert '"source_timeout": 0.75' in launch
    assert '"front_depth.topic": "/safety_test/points"' in launch
