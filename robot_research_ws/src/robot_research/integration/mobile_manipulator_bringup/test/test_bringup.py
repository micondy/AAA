from pathlib import Path
import ast

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from mobile_manipulator_bringup.base_io_adapter import (
    odometry_to_transform,
    twist_is_finite,
)
from mobile_manipulator_bringup.moveit_collision_validation import (
    CASES,
    arm_joint_names,
    contact_bodies,
)
from mobile_manipulator_bringup.task_scene import load_task_scene
from moveit_msgs.msg import ContactInformation


def test_twist_validation_rejects_non_finite_commands():
    message = Twist()
    assert twist_is_finite(message)
    message.linear.y = float("nan")
    assert not twist_is_finite(message)


def test_odometry_transform_preserves_frames_pose_and_stamp():
    message = Odometry()
    message.header.frame_id = "odom"
    message.header.stamp.sec = 12
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.4
    message.pose.pose.orientation.z = 0.2
    message.pose.pose.orientation.w = 0.98

    transform = odometry_to_transform(message)
    assert transform.header.frame_id == "odom"
    assert transform.header.stamp.sec == 12
    assert transform.child_frame_id == "base_link"
    assert transform.transform.translation.x == 1.25
    assert transform.transform.translation.y == -0.4
    assert transform.transform.rotation.z == 0.2
    assert transform.transform.rotation.w == 0.98


def test_mock_launch_selects_mobile_tf_and_base_controller():
    launch_file = Path(__file__).parents[1] / "launch" / "mock.launch.py"
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)
    assert '"fixed_base": "false"' in source
    assert '"start_base_controller": "true"' in source
    assert '"publish_odom_tf": True' in source
    assert '"publish_ground_truth": True' in source


def test_sim_perception_profile_stays_thin_and_owns_camera_tf():
    launch_file = Path(__file__).parents[1] / "launch" / "sim_perception.launch.py"
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)

    assert '" nav_camera_use_nominal_extrinsics:=true"' in source
    assert 'package="robot_state_publisher"' in source
    assert 'package="target_perception"' in source
    assert '"--enable-nav-rgbd"' in source
    assert 'DeclareLaunchArgument("start_isaac", default_value="true")' in source
    assert 'DeclareLaunchArgument("publish_world_tf", default_value="false")' in source
    assert 'DeclareLaunchArgument("green_target_depth", default_value="0.77")' in source
    assert 'DeclareLaunchArgument("run_benchmark", default_value="false")' in source
    assert 'DeclareLaunchArgument("run_distance_benchmark", default_value="false")' in source
    assert 'duration = len(_distance_values(context)) * stage_seconds + 1.0' in source
    assert "move_group" not in source
    assert "nav2" not in source.lower()
    assert "realsense2_camera" not in source


def test_real_instance_perception_uses_one_d435i_and_general_model():
    package = Path(__file__).parents[1]
    launch_file = package / "launch" / "real_instance_perception.launch.py"
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)

    assert 'get_package_share_directory("d435i_bringup")' in source
    assert 'get_package_share_directory("experiment_tools")' in source
    assert '"camera_namespace": "sensors"' in source
    assert '"camera_name": "nav_camera"' in source
    assert 'DeclareLaunchArgument("start_camera", default_value="true")' in source
    assert 'executable="instance_segmentation"' in source
    assert 'executable="instance_tracking"' in source
    assert 'default_value="nav_camera_color_optical_frame"' in source
    assert 'default_value="aligned"' in source
    assert 'package="rviz2"' in source
    assert 'executable="general_object_benchmark"' in source
    assert 'DeclareLaunchArgument("run_benchmark", default_value="false")' in source
    assert 'DeclareLaunchArgument("record_bag", default_value="false")' in source


def test_collision_validation_cases_are_complete_and_distinct():
    assert arm_joint_names("left") == tuple(
        f"openarm_left_joint{index}" for index in range(1, 8)
    )
    assert len({case.name for case in CASES}) == len(CASES)
    assert {case.expected_valid for case in CASES} == {True, False}
    assert any(
        case.expected_contact is not None
        and "work_surface" in case.expected_contact
        for case in CASES
    )
    for case in CASES:
        assert len(case.joint_names) == 7
        assert len(case.positions) == 7


def test_contact_pair_comparison_is_order_independent():
    contact = ContactInformation()
    contact.contact_body_1 = "openarm_left_link4"
    contact.contact_body_2 = "openarm_body_link0"
    assert contact_bodies(contact) == frozenset(
        ("openarm_body_link0", "openarm_left_link4")
    )


def test_generic_task_scene_has_valid_unique_boxes():
    package = Path(__file__).parents[1]
    scene = load_task_scene(package / "config" / "generic_task_scene.yaml")
    assert scene.frame_id == "world"
    assert {box.object_id for box in scene.objects} == {
        "work_surface",
        "shelf_back",
        "shelf_upper",
        "occluder",
    }
    assert all(all(size > 0.0 for size in box.size) for box in scene.objects)


def test_task_scene_launch_is_fixed_base_and_does_not_start_base_control():
    launch_file = Path(__file__).parents[1] / "launch" / "task_scene.launch.py"
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)
    assert '"fixed_base": "true"' in source
    assert '"start_base_controller": "false"' in source
    assert 'executable="apply_task_scene"' in source


def test_sim_target_belief_composes_thin_perception_and_belief_only():
    launch_file = Path(__file__).parents[1] / "launch" / "sim_target_belief.launch.py"
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)
    assert '"sim_perception.launch.py"' in source
    assert '"belief.launch.py"' in source
    assert '"run_belief_benchmark"' in source
    assert '"run_benchmark": "false"' in source
    assert '"run_distance_benchmark": "false"' in source
    assert "move_group" not in source
    assert "nav2" not in source.lower()


def test_sim_active_perception_composes_one_navigation_and_nbv_stack():
    package = Path(__file__).parents[1]
    launch_file = package / "launch" / "sim_active_perception.launch.py"
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source)

    assert '"mobile_base_sim_navigation.launch.py"' in source
    assert 'executable="color_depth_detector"' in source
    assert 'executable="instance_tracking"' in source
    assert '"active_perception_stack.launch.py"' in source
    assert '"belief_frame": "odom"' in source
    assert '"output_frame": "odom"' in source
    assert '"max_missed_frames": 1500' in source
    assert '"target_max_missed_frames": "1500"' in source
    assert '"map_topic": "/rtabmap/map"' in source
    assert '"execution_enabled": LaunchConfiguration("enable_nbv_execution")' in source
    assert 'DeclareLaunchArgument("enable_nbv_execution", default_value="false")' in source
    assert '"hsv_lower": [50, 150, 80]' in source
    assert '"hsv_upper": [63, 255, 255]' in source
    assert '"use_rviz": "false"' in source
    assert 'name="active_perception_rviz"' in source
    assert "robot_state_publisher" not in source
    assert "realsense2_camera" not in source

    rviz = package / "rviz" / "active_perception_navigation.rviz"
    rviz_text = rviz.read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz_text
    assert "/active_perception/candidate_view_markers" in rviz_text
    assert "/active_perception/best_view_pose" in rviz_text
