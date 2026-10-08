#!/usr/bin/env python3
"""Load and validate the project's four-wheel-steering base in Isaac Sim.

The default mode keeps every joint stopped for physical-model inspection.
``--demo`` adds a deliberately slow straight/rotate-in-place motion test.  ROS,
cameras and OpenArm remain separate milestones so failures stay attributable.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run a slow straight and rotate-in-place chassis demonstration.",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="Subscribe to /cmd_vel and publish clock, joint and ground-truth state.",
    )
    parser.add_argument(
        "--enable-nav-rgbd",
        action="store_true",
        help="Publish the chassis-fixed D435i-equivalent RGB-D contract.",
    )
    parser.add_argument(
        "--test-seconds",
        type=float,
        default=0.0,
        help="Run for this many simulated seconds and then exit; 0 keeps the GUI open.",
    )
    args, _ = parser.parse_known_args()
    if args.demo and args.ros:
        parser.error("--demo and --ros are mutually exclusive command sources")
    return args


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils
import carb
import omni.graph.core as og
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdGeom


WORKSPACE = Path(__file__).resolve().parents[2]
ROBOT_USD = (
    WORKSPACE
    / "assets/mobile_base/cad_20260912/mobile_base_sim/mobile_base_sim.usda"
)
ROBOT_PRIM = "/World/MobileBase"
BASE_LINK_PRIM = ROBOT_PRIM + "/Geometry/base_link"
STEERING_JOINTS = (
    "front_left_steer_joint",
    "front_right_steer_joint",
    "rear_left_steer_joint",
    "rear_right_steer_joint",
)
DRIVE_JOINTS = (
    "front_left_drive_joint",
    "front_right_drive_joint",
    "rear_left_drive_joint",
    "rear_right_drive_joint",
)
MODULE_X = (0.2095, 0.2095, -0.2095, -0.2095)
MODULE_Y = (0.225, -0.225, 0.225, -0.225)
WHEEL_RADIUS = 0.0855
COMMAND_TIMEOUT_S = 0.25
MAX_LINEAR_SPEED = 0.30
MAX_LATERAL_SPEED = 0.20
MAX_ANGULAR_SPEED = 0.50
ROS_GRAPH = "/MobileBaseRosGraph"
NAV_CAMERA_GRAPH = "/MobileBaseNavCameraGraph"
NAV_CAMERA_PRIM = BASE_LINK_PRIM + "/nav_camera_color"
# Official realsense2_description nominal color optical center for a mount at
# base_link xyz="0.32 0 0.42".
NAV_CAMERA_TRANSLATION = (0.3306, 0.0325, 0.4325)


class RosCmdVelInput:
    """Small ROS input boundary with finite-value checks and a dead-man timeout."""

    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import Twist

        self._rclpy = rclpy
        rclpy.init()
        self._node = rclpy.create_node("mobile_base_isaac_sim")
        self._command = (0.0, 0.0, 0.0)
        self._last_message_time = 0.0
        self._received_once = False
        self._subscription = self._node.create_subscription(
            Twist, "/cmd_vel", self._on_command, 10
        )

    def _on_command(self, message: object) -> None:
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            self._node.get_logger().error("Rejected non-finite /cmd_vel")
            self._command = (0.0, 0.0, 0.0)
        else:
            vx = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, values[0]))
            vy = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, values[1]))
            length = math.hypot(vx, vy)
            if length > MAX_LINEAR_SPEED:
                vx *= MAX_LINEAR_SPEED / length
                vy *= MAX_LINEAR_SPEED / length
            wz = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, values[2]))
            self._command = (vx, vy, wz)
        self._last_message_time = time.monotonic()
        if not self._received_once:
            print("[mobile_base_sim] first /cmd_vel received", flush=True)
            self._received_once = True

    def command(self) -> tuple[float, float, float]:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        if time.monotonic() - self._last_message_time > COMMAND_TIMEOUT_S:
            return 0.0, 0.0, 0.0
        return self._command

    def close(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()


def set_float(prim: Usd.Prim, name: str, value: float) -> None:
    attribute = prim.GetAttribute(name)
    if not attribute.IsValid():
        attribute = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
    attribute.Set(value)


def configure_joint_drives(stage: Usd.Stage) -> None:
    for name in STEERING_JOINTS:
        prim = stage.GetPrimAtPath(f"{ROBOT_PRIM}/Physics/{name}")
        if not prim.IsValid():
            raise RuntimeError(f"Missing steering joint: {name}")
        set_float(prim, "drive:angular:physics:stiffness", 400.0)
        set_float(prim, "drive:angular:physics:damping", 40.0)
        set_float(prim, "drive:angular:physics:maxForce", 150.0)
        set_float(prim, "drive:angular:physics:targetPosition", 0.0)

    for name in DRIVE_JOINTS:
        prim = stage.GetPrimAtPath(f"{ROBOT_PRIM}/Physics/{name}")
        if not prim.IsValid():
            raise RuntimeError(f"Missing drive joint: {name}")
        set_float(prim, "drive:angular:physics:stiffness", 0.0)
        set_float(prim, "drive:angular:physics:damping", 5.0)
        set_float(prim, "drive:angular:physics:maxForce", 80.0)
        set_float(prim, "drive:angular:physics:targetVelocity", 0.0)


def create_ros_state_graph() -> None:
    """Publish simulation time and non-authoritative ground-truth state.

    Ground-truth TF deliberately uses a separate topic.  RTAB-Map remains the
    sole owner of the standard ``odom -> base_link`` transform when mapping or
    localizing.
    """
    import usdrt.Sdf

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": ROS_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishGroundTruthTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(BASE_LINK_PRIM)]),
                ("PublishOdometry.inputs:topicName", "sim/ground_truth/odom"),
                ("PublishOdometry.inputs:odomFrameId", "sim_world"),
                ("PublishOdometry.inputs:chassisFrameId", "base_link"),
                ("PublishOdometry.inputs:publishRawVelocities", False),
                ("PublishGroundTruthTf.inputs:topicName", "sim/ground_truth/tf"),
                ("PublishGroundTruthTf.inputs:parentFrameId", "sim_world"),
                ("PublishGroundTruthTf.inputs:childFrameId", "base_link"),
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(BASE_LINK_PRIM)]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:execOut", "PublishGroundTruthTf.inputs:execIn"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishGroundTruthTf.inputs:timeStamp"),
                ("ComputeOdometry.outputs:position", "PublishGroundTruthTf.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishGroundTruthTf.inputs:rotation"),
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                (
                    "ReadJointState.outputs:jointPositions",
                    "PublishJointState.inputs:jointPositions",
                ),
                (
                    "ReadJointState.outputs:jointVelocities",
                    "PublishJointState.inputs:jointVelocities",
                ),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                (
                    "ReadJointState.outputs:stageMetersPerUnit",
                    "PublishJointState.inputs:stageMetersPerUnit",
                ),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    print(
        "[mobile_base_sim] ROS state READY topics=/clock,/joint_states,"
        "/sim/ground_truth/odom,/sim/ground_truth/tf command=/cmd_vel",
        flush=True,
    )


def create_nav_camera(stage: Usd.Stage) -> None:
    """Create the fixed forward RGB-D sensor with the project's ROS contract."""
    import usdrt.Sdf

    camera = UsdGeom.Camera.Define(stage, NAV_CAMERA_PRIM)
    camera.AddTranslateOp().Set(Gf.Vec3d(*NAV_CAMERA_TRANSLATION))
    # USD camera -Z forward/+Y up -> ROS optical +Z forward/-Y up.
    camera.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.10, 10.0))
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(20.955 * 480.0 / 640.0)
    camera.GetFocalLengthAttr().Set(10.45)
    camera_prim = camera.GetPrim()
    camera_prim.ApplyAPI("OmniSensorAPI")
    camera_prim.GetAttribute("omni:sensor:tickRate").Set(15.0)

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": NAV_CAMERA_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RgbPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PointcloudPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublisher", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(NAV_CAMERA_PRIM)]),
                ("CreateRenderProduct.inputs:width", 640),
                ("CreateRenderProduct.inputs:height", 480),
                ("RgbPublisher.inputs:topicName", "sensors/nav_camera/color/image_raw"),
                ("RgbPublisher.inputs:type", "rgb"),
                ("RgbPublisher.inputs:frameId", "nav_camera_color_optical_frame"),
                ("RgbPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "DepthPublisher.inputs:topicName",
                    "sensors/nav_camera/aligned_depth_to_color/image_raw",
                ),
                ("DepthPublisher.inputs:type", "depth"),
                ("DepthPublisher.inputs:frameId", "nav_camera_color_optical_frame"),
                ("DepthPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "PointcloudPublisher.inputs:topicName",
                    "sensors/nav_camera/depth/color/points",
                ),
                ("PointcloudPublisher.inputs:type", "depth_pcl"),
                ("PointcloudPublisher.inputs:frameId", "nav_camera_color_optical_frame"),
                ("PointcloudPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "CameraInfoPublisher.inputs:topicName",
                    "sensors/nav_camera/color/camera_info",
                ),
                ("CameraInfoPublisher.inputs:frameId", "nav_camera_color_optical_frame"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PointcloudPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfoPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RgbPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PointcloudPublisher.inputs:renderProductPath"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "CameraInfoPublisher.inputs:renderProductPath",
                ),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    print(
        "[mobile_base_sim] nav RGB-D READY 640x480@15Hz "
        "frame=nav_camera_color_optical_frame",
        flush=True,
    )


def swerve_targets(vx: float, vy: float, wz: float) -> tuple[list[float], list[float]]:
    """Match the real controller's per-module inverse kinematics.

    Steering is limited to +/-90 degrees.  When the raw direction is outside
    that range, the wheel turns to the equivalent direction and reverses.
    """
    steering: list[float] = []
    wheel_speed: list[float] = []
    for x, y in zip(MODULE_X, MODULE_Y):
        module_vx = vx - wz * y
        module_vy = vy + wz * x
        speed = math.hypot(module_vx, module_vy)
        if speed < 1.0e-6:
            steering.append(0.0)
            wheel_speed.append(0.0)
            continue
        angle = math.atan2(module_vy, module_vx)
        if angle > math.pi / 2.0:
            angle -= math.pi
            speed = -speed
        elif angle < -math.pi / 2.0:
            angle += math.pi
            speed = -speed
        steering.append(angle)
        wheel_speed.append(speed / WHEEL_RADIUS)
    return steering, wheel_speed


def command_joints(
    stage: Usd.Stage, steering_rad: list[float], wheel_rad_s: list[float]
) -> None:
    """Set USD angular drive targets (USD stores angular targets in degrees)."""
    for name, angle in zip(STEERING_JOINTS, steering_rad):
        set_float(
            stage.GetPrimAtPath(f"{ROBOT_PRIM}/Physics/{name}"),
            "drive:angular:physics:targetPosition",
            math.degrees(angle),
        )
    for name, speed in zip(DRIVE_JOINTS, wheel_rad_s):
        set_float(
            stage.GetPrimAtPath(f"{ROBOT_PRIM}/Physics/{name}"),
            "drive:angular:physics:targetVelocity",
            math.degrees(speed),
        )


def demo_command(seconds: float) -> tuple[str, list[float], list[float]]:
    """Return a low-speed, finite demonstration phase for the current time."""
    if seconds < 1.0:
        return "settle", *swerve_targets(0.0, 0.0, 0.0)
    if seconds < 4.0:
        return "straight 0.15 m/s", *swerve_targets(0.15, 0.0, 0.0)
    if seconds < 5.0:
        return "stop", *swerve_targets(0.0, 0.0, 0.0)
    if seconds < 7.0:
        steering, _ = swerve_targets(0.0, 0.0, 0.25)
        return "steer for rotation", steering, [0.0] * 4
    if seconds < 11.0:
        return "rotate 0.25 rad/s", *swerve_targets(0.0, 0.0, 0.25)
    return "finished", *swerve_targets(0.0, 0.0, 0.0)


def base_pose(stage: Usd.Stage) -> tuple[Gf.Vec3d, Gf.Quatd]:
    base_link = stage.GetPrimAtPath(BASE_LINK_PRIM)
    transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        base_link
    )
    return transform.ExtractTranslation(), transform.ExtractRotationQuat()


def main() -> None:
    if not ROBOT_USD.is_file():
        raise RuntimeError(f"Imported robot USD not found: {ROBOT_USD}")

    ros_input = None
    ros_enabled = args.ros or args.enable_nav_rgbd
    if ros_enabled:
        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        carb.settings.get_settings().set_bool(
            "/exts/isaacsim.ros2.bridge/publish_without_verification", True
        )

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Isaac Sim assets root is unavailable")

    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Environments/Grid/default_environment.usd",
        path="/World/Ground",
    )
    robot_prim = stage_utils.add_reference_to_stage(
        usd_path=str(ROBOT_USD), path=ROBOT_PRIM
    )
    UsdGeom.Xformable(robot_prim).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.15))

    stage = robot_prim.GetStage()
    configure_joint_drives(stage)
    if ros_enabled:
        create_ros_state_graph()
    if args.enable_nav_rgbd:
        create_nav_camera(stage)
    if args.ros:
        ros_input = RosCmdVelInput()
    set_camera_view(
        eye=[1.7, 1.7, 1.2],
        target=[0.0, 0.0, 0.15],
        camera_prim_path="/OmniverseKit_Persp",
    )

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    simulation_app.update()

    initial_position, _ = base_pose(stage)
    print(
        f"[mobile_base_sim] READY initial base position={tuple(initial_position)}",
        flush=True,
    )

    step = 0
    previous_phase = ""
    held_steering = [0.0] * 4
    steering_ready_at = 0.0
    max_steps = int(args.test_seconds * 60.0) if args.test_seconds > 0.0 else None
    try:
        while simulation_app.is_running():
            seconds = step / 60.0
            if args.demo:
                phase, steering, wheel_speed = demo_command(seconds)
                command_joints(stage, steering, wheel_speed)
                if phase != previous_phase:
                    print(f"[mobile_base_sim] demo phase: {phase}", flush=True)
                    previous_phase = phase
            elif ros_input is not None:
                vx, vy, wz = ros_input.command()
                if math.hypot(vx, vy) < 1.0e-4 and abs(wz) < 1.0e-4:
                    command_joints(stage, held_steering, [0.0] * 4)
                else:
                    steering, wheel_speed = swerve_targets(vx, vy, wz)
                    steering_change = max(
                        abs(target - held)
                        for target, held in zip(steering, held_steering)
                    )
                    if steering_change > math.radians(5.0):
                        held_steering = steering
                        steering_ready_at = seconds + steering_change / 0.5 + 0.15
                    command_joints(
                        stage,
                        held_steering,
                        wheel_speed if seconds >= steering_ready_at else [0.0] * 4,
                    )
            simulation_app.update()
            step += 1
            if step % 300 == 0:
                position, orientation = base_pose(stage)
                print(
                    "[mobile_base_sim] "
                    f"t={step / 60.0:.1f}s position={tuple(position)} "
                    f"orientation={orientation}",
                    flush=True,
                )
            if max_steps is not None and step >= max_steps:
                break
    finally:
        command_joints(stage, held_steering, [0.0] * 4)
        if ros_input is not None:
            ros_input.close()


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    app_utils.stop()
    simulation_app.close()
