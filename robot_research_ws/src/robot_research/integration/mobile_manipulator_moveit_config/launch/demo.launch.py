"""Safe MoveIt 2 mock demo for the integrated mobile manipulator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    combined_description = Path(
        get_package_share_directory("mobile_manipulator_description")
    ) / "urdf" / "mobile_manipulator.urdf.xacro"
    official_moveit = Path(
        get_package_share_directory("openarm_bimanual_moveit_config")
    ) / "config"
    official_controllers = Path(
        get_package_share_directory("openarm_bringup")
    ) / "config" / "v10_controllers" / "openarm_v10_bimanual_controllers.yaml"
    base_controllers = Path(
        get_package_share_directory("mobile_base_bringup")
    ) / "config" / "controllers.yaml"
    local_config = Path(
        get_package_share_directory("mobile_manipulator_moveit_config")
    ) / "config"

    # The mock entry point deliberately cannot select the real CAN hardware plugin.
    xacro_mappings = {
        "use_mock_hardware": "true",
        "openarm_ros2_control": "true",
        "openarm_use_fake_hardware": "true",
        "hand": "true",
        # Real/mock ROS profiles use the driver-calibrated internal TF tree.
        # Only Isaac Sim requests nominal optical frames from the URDF.
        "nav_camera_enabled": "true",
        "nav_camera_use_nominal_extrinsics": "false",
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "mobile_manipulator",
            package_name="mobile_manipulator_moveit_config",
        )
        .robot_description(
            file_path=str(combined_description),
            mappings=xacro_mappings,
        )
        .robot_description_semantic(
            file_path="config/mobile_manipulator.srdf.xacro"
        )
        .robot_description_kinematics(
            file_path=str(official_moveit / "kinematics.yaml")
        )
        # Upstream disables acceleration limits, which makes MoveIt's time
        # parameterization reject otherwise valid paths. Keep that fix local.
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path=str(official_moveit / "moveit_controllers.yaml")
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits(
            file_path=str(official_moveit / "pilz_cartesian_limits.yaml")
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    use_rviz = LaunchConfiguration("use_rviz")
    fixed_base = LaunchConfiguration("fixed_base")
    start_base_controller = LaunchConfiguration("start_base_controller")

    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
        output="screen",
        condition=IfCondition(fixed_base),
    )

    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "odom",
        ],
        output="screen",
        condition=UnlessCondition(fixed_base),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        # The two files remain owned by their upstream bringup packages. Loading
        # both lets one controller manager own the complete integrated robot.
        parameters=[
            moveit_config.robot_description,
            str(official_controllers),
            str(base_controllers),
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="log",
        arguments=["-d", str(local_config / "moveit.rviz")],
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(use_rviz),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_joint_trajectory_controller",
            "right_joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )
    gripper_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "right_gripper_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )
    base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_controller", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(start_base_controller),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "fixed_base",
                default_value="true",
                description="Publish world->base_link for stationary MoveIt demos",
            ),
            DeclareLaunchArgument(
                "start_base_controller",
                default_value="false",
                description="Activate the four-wheel-steering controller",
            ),
            world_to_base,
            world_to_odom,
            robot_state_publisher,
            ros2_control,
            move_group,
            rviz,
            TimerAction(period=1.0, actions=[joint_state_broadcaster]),
            TimerAction(period=2.0, actions=[arm_controllers, gripper_controllers]),
            TimerAction(period=3.0, actions=[base_controller]),
        ]
    )
