#!/usr/bin/env python3
"""Validate the custom base plus NVIDIA official bimanual OpenArm asset."""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path
import sys
import time
import traceback

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--empty-scene",
        action="store_true",
        help="Use only the grid floor instead of the indoor mapping scene.",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="Receive /cmd_vel and publish base state for RTAB-Map/Nav2.",
    )
    parser.add_argument(
        "--enable-nav-rgbd",
        action="store_true",
        help="Publish the provisional top-mounted navigation D435 RGB-D contract.",
    )
    parser.add_argument(
        "--enable-multi-sensor-nav",
        action="store_true",
        help="Publish four RGB-D cameras and two RTX 2-D LiDAR scans.",
    )
    parser.add_argument("--nav-camera-width", type=int, default=640)
    parser.add_argument("--nav-camera-height", type=int, default=480)
    parser.add_argument("--nav-camera-hz", type=float, default=15.0)
    parser.add_argument(
        "--enable-nav-obstacle-control",
        action="store_true",
        help=(
            "Expose services for standard, low, hanging and shelf navigation "
            "obstacles placed 0.75 m in front of the current base."
        ),
    )
    parser.add_argument(
        "--add-green-target",
        action="store_true",
        help="Add a 0.20 m green cube for target_perception validation.",
    )
    parser.add_argument(
        "--add-rack",
        action="store_true",
        help="Add the three-layer rack and publish semantic labels.",
    )
    parser.add_argument(
        "--rack-asset",
        default="",
        help="Converted rack USD; empty uses assets/rack/Gupeng.usd.",
    )
    parser.add_argument(
        "--rack-layout",
        choices=("grid4x4", "single", "cad"),
        default="grid4x4",
        help="Rack scene: navigable 4x4 units (default), one unit, or the full CAD.",
    )
    parser.add_argument(
        "--rack-visual-seed", type=int, default=0,
        help="0 keeps baseline colors; positive seeds vary rack colors for synthetic data.",
    )
    parser.add_argument(
        "--parametric-rack",
        action="store_true",
        help="Deprecated alias for --rack-layout single.",
    )
    parser.add_argument(
        "--green-target-depth",
        type=float,
        default=0.77,
        help=(
            "Distance in meters from the nav camera optical center to the "
            "cube's visible front surface."
        ),
    )
    parser.add_argument(
        "--green-target-depth-sequence",
        type=float,
        nargs="+",
        help="Move the target through these front-surface depths in one process.",
    )
    parser.add_argument(
        "--green-target-stage-seconds",
        type=float,
        default=10.0,
        help="Simulated seconds to hold each target depth in a sequence.",
    )
    parser.add_argument(
        "--test-seconds",
        type=float,
        default=0.0,
        help="Exit after this many simulated seconds; headless defaults to 10.",
    )
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=1.0,
        help=(
            "Maximum simulated-time/wall-time ratio. The default 1.0 keeps "
            "ROS camera and TF queues bounded; 0 disables throttling."
        ),
    )
    return parser.parse_known_args()[0]


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.semantics as semantics_utils
import carb
import omni
import omni.graph.core as og
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


WORKSPACE = Path(__file__).resolve().parents[2]
ROBOT_USD = WORKSPACE / "assets/mobile_manipulator/mobile_manipulator_official.usda"
ROBOT_PRIM = "/World/MobileManipulator"
BASE_PRIM = ROBOT_PRIM + "/MobileBase"
OPENARM_PRIM = ROBOT_PRIM + "/OpenArm"
BASE_LINK_PRIM = BASE_PRIM + "/Geometry/base_link"
ROS_GRAPH = "/MobileManipulatorBaseRosGraph"
NAV_CAMERA_OPTICAL_OFFSET = (0.0106, 0.0325, 0.0125)
CAMERA_SPECS = (
    # name, base_link translation, pitch down, yaw about base +Z
    ("nav_camera", (0.10, 0.0, 1.70), math.radians(12.0), 0.0),
    ("rear_camera", (-0.25, 0.0, 0.65), 0.0, math.pi),
    ("left_camera", (0.0, 0.28, 0.65), 0.0, math.pi / 2.0),
    ("right_camera", (0.0, -0.28, 0.65), 0.0, -math.pi / 2.0),
)
LIDAR_SPECS = (
    ("front_lidar", (0.30, 0.0, 0.32)),
    ("rear_lidar", (-0.30, 0.0, 0.32)),
)
GREEN_TARGET_PRIM = "/World/GreenTarget"
GREEN_TARGET_SIZE = 0.20
RACK_PRIM = "/World/ValidationRack"
DEFAULT_RACK_USD = WORKSPACE / "assets/rack/Gupeng.usd"
NAV_OBSTACLE_PRIM = "/World/NavTestObstacle"
NAV_OBSTACLE_DISTANCE = 1.00
NAV_OBSTACLE_LOW_DISTANCE = 1.00
NAV_OBSTACLE_PRIMS = {
    "standard": NAV_OBSTACLE_PRIM,
    "low": "/World/NavTestObstacleLow",
    "hanging": "/World/NavTestObstacleHanging",
    "shelf": "/World/NavTestObstacleShelf",
}
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
STEERING_ALIGNMENT_TOLERANCE = math.radians(5.0)
MAX_LINEAR_SPEED = 0.30
MAX_LATERAL_SPEED = 0.30
MAX_ANGULAR_SPEED = 0.50
# The imported wheel/contact model has a static-friction dead zone below about
# 0.05 rad/s.  Compensate only pure in-place turns in simulation; real chassis
# drivers remain responsible for their own actuator dead-zone compensation.
MIN_SIM_PURE_ROTATION_SPEED = 0.12
OPENARM_ARM_JOINTS = tuple(
    f"openarm_{side}_joint{index}"
    for side in ("left", "right")
    for index in range(1, 8)
)


class RosCmdVelInput:
    def __init__(self, obstacle_control: bool = False) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import JointState
        from std_srvs.srv import SetBool

        self._rclpy = rclpy
        rclpy.init()
        self._node = rclpy.create_node("mobile_manipulator_isaac_sim")
        self._command = (0.0, 0.0, 0.0)
        self._last_message_time = 0.0
        self._received_motion_command = False
        self._steering_positions: tuple[float, float, float, float] | None = None
        self._obstacle_requests: list[tuple[str, bool]] = []
        self._node.create_subscription(Twist, "/cmd_vel", self._on_command, 10)
        self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        if obstacle_control:
            self._node.create_service(
                SetBool,
                "/sim/nav_obstacle/set_enabled",
                self._make_obstacle_callback("standard"),
            )
            for obstacle_type in NAV_OBSTACLE_PRIMS:
                self._node.create_service(
                    SetBool,
                    f"/sim/nav_obstacle/{obstacle_type}/set_enabled",
                    self._make_obstacle_callback(obstacle_type),
                )

    def _on_command(self, message: object) -> None:
        values = (float(message.linear.x), float(message.linear.y), float(message.angular.z))
        if not all(math.isfinite(value) for value in values):
            self._command = (0.0, 0.0, 0.0)
            return
        vx = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, values[0]))
        vy = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, values[1]))
        length = math.hypot(vx, vy)
        if length > MAX_LINEAR_SPEED:
            vx *= MAX_LINEAR_SPEED / length
            vy *= MAX_LINEAR_SPEED / length
        wz = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, values[2]))
        if (
            math.hypot(vx, vy) < 1.0e-4
            and 1.0e-4 <= abs(wz) < MIN_SIM_PURE_ROTATION_SPEED
        ):
            wz = math.copysign(MIN_SIM_PURE_ROTATION_SPEED, wz)
        self._command = (vx, vy, wz)
        if math.hypot(vx, vy) >= 1.0e-4 or abs(wz) >= 1.0e-4:
            self._received_motion_command = True
        self._last_message_time = time.monotonic()

    def _on_joint_state(self, message: object) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in STEERING_JOINTS):
            self._steering_positions = tuple(
                float(positions[name]) for name in STEERING_JOINTS
            )

    def steering_aligned(self, targets: list[float]) -> bool:
        if self._steering_positions is None:
            return False
        return all(
            abs(math.remainder(actual - target, 2.0 * math.pi))
            <= STEERING_ALIGNMENT_TOLERANCE
            for actual, target in zip(self._steering_positions, targets)
        )

    @property
    def received_motion_command(self) -> bool:
        return self._received_motion_command

    def command(self) -> tuple[float, float, float]:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        if time.monotonic() - self._last_message_time > COMMAND_TIMEOUT_S:
            return 0.0, 0.0, 0.0
        return self._command

    def _make_obstacle_callback(self, obstacle_type: str):
        def callback(request: object, response: object) -> object:
            self._obstacle_requests.append((obstacle_type, bool(request.data)))
            response.success = True
            response.message = f"{obstacle_type} navigation obstacle update queued"
            return response

        return callback

    def take_obstacle_request(self) -> tuple[str, bool] | None:
        if not self._obstacle_requests:
            return None
        return self._obstacle_requests.pop(0)

    def close(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()


def set_float(prim: Usd.Prim, name: str, value: float) -> None:
    attribute = prim.GetAttribute(name)
    if not attribute.IsValid():
        attribute = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
    attribute.Set(value)


def require_prim(stage: Usd.Stage, path: str) -> Usd.Prim:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing required prim: {path}")
    return prim


def configure_base_joint_drives(stage: Usd.Stage) -> None:
    for name in STEERING_JOINTS:
        prim = require_prim(stage, f"{BASE_PRIM}/Physics/{name}")
        set_float(prim, "drive:angular:physics:stiffness", 400.0)
        set_float(prim, "drive:angular:physics:damping", 40.0)
        set_float(prim, "drive:angular:physics:maxForce", 150.0)
        set_float(prim, "drive:angular:physics:targetPosition", 0.0)

    for name in DRIVE_JOINTS:
        prim = require_prim(stage, f"{BASE_PRIM}/Physics/{name}")
        set_float(prim, "drive:angular:physics:stiffness", 0.0)
        set_float(prim, "drive:angular:physics:damping", 5.0)
        set_float(prim, "drive:angular:physics:maxForce", 80.0)
        set_float(prim, "drive:angular:physics:targetVelocity", 0.0)


def swerve_targets(vx: float, vy: float, wz: float) -> tuple[list[float], list[float]]:
    steering = []
    wheel_speed = []
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


def command_base(stage: Usd.Stage, steering: list[float], wheel_speed: list[float]) -> None:
    for name, angle in zip(STEERING_JOINTS, steering):
        set_float(
            require_prim(stage, f"{BASE_PRIM}/Physics/{name}"),
            "drive:angular:physics:targetPosition",
            math.degrees(angle),
        )
    for name, speed in zip(DRIVE_JOINTS, wheel_speed):
        set_float(
            require_prim(stage, f"{BASE_PRIM}/Physics/{name}"),
            "drive:angular:physics:targetVelocity",
            math.degrees(speed),
        )


def inverse_transform(position, rotation_xyzw):
    """Return the inverse rigid transform as translation and xyzw quaternion."""
    x, y, z, w = (float(value) for value in rotation_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("Cannot invert a zero-length quaternion")
    x, y, z, w = -x / norm, -y / norm, -z / norm, w / norm
    vx, vy, vz = (-float(value) for value in position)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    translation = [
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    ]
    return translation, [x, y, z, w]


def create_ros_state_graph(stage: Usd.Stage) -> None:
    """Publish time, idealized controller odometry, and separate ground truth.

    The simulation controller odometry owns ``odom -> base_link`` just like the
    real chassis controller.  Ground truth stays on dedicated topics so it can
    be used for evaluation without becoming an input to localization.
    """
    import usdrt.Sdf

    keys = og.Controller.Keys
    initial_position, initial_orientation = base_pose(stage)
    initial_imaginary = initial_orientation.GetImaginary()
    initial_rotation_xyzw = [
        float(initial_imaginary[0]),
        float(initial_imaginary[1]),
        float(initial_imaginary[2]),
        float(initial_orientation.GetReal()),
    ]
    odom_world_translation, odom_world_rotation = inverse_transform(
        initial_position, initial_rotation_xyzw
    )
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": ROS_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("JointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Odometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("ClockPub", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("GroundTruthOdomPub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("ControllerOdomPub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("OdomTfPub", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("WorldOdomTfPub", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("GroundTruthTfPub", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("JointPub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("JointState.inputs:prim", [usdrt.Sdf.Path(BASE_LINK_PRIM)]),
                ("Odometry.inputs:chassisPrim", [usdrt.Sdf.Path(BASE_LINK_PRIM)]),
                ("GroundTruthOdomPub.inputs:topicName", "sim/ground_truth/odom"),
                ("GroundTruthOdomPub.inputs:odomFrameId", "sim_world"),
                ("GroundTruthOdomPub.inputs:chassisFrameId", "base_link"),
                ("GroundTruthOdomPub.inputs:publishRawVelocities", False),
                ("ControllerOdomPub.inputs:topicName", "swerve_controller/odom"),
                ("ControllerOdomPub.inputs:odomFrameId", "odom"),
                ("ControllerOdomPub.inputs:chassisFrameId", "base_link"),
                ("ControllerOdomPub.inputs:publishRawVelocities", False),
                ("OdomTfPub.inputs:topicName", "tf"),
                ("OdomTfPub.inputs:parentFrameId", "odom"),
                ("OdomTfPub.inputs:childFrameId", "base_link"),
                ("WorldOdomTfPub.inputs:topicName", "tf"),
                ("WorldOdomTfPub.inputs:parentFrameId", "odom"),
                ("WorldOdomTfPub.inputs:childFrameId", "sim_world"),
                ("WorldOdomTfPub.inputs:translation", odom_world_translation),
                ("WorldOdomTfPub.inputs:rotation", odom_world_rotation),
                ("GroundTruthTfPub.inputs:topicName", "sim/ground_truth/tf"),
                ("GroundTruthTfPub.inputs:parentFrameId", "sim_world"),
                ("GroundTruthTfPub.inputs:childFrameId", "base_link"),
                ("JointPub.inputs:topicName", "joint_states"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                ("Tick.outputs:tick", "Odometry.inputs:execIn"),
                ("Odometry.outputs:execOut", "GroundTruthOdomPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "GroundTruthOdomPub.inputs:timeStamp"),
                ("Odometry.outputs:position", "GroundTruthOdomPub.inputs:position"),
                ("Odometry.outputs:orientation", "GroundTruthOdomPub.inputs:orientation"),
                ("Odometry.outputs:linearVelocity", "GroundTruthOdomPub.inputs:linearVelocity"),
                ("Odometry.outputs:angularVelocity", "GroundTruthOdomPub.inputs:angularVelocity"),
                ("Odometry.outputs:execOut", "ControllerOdomPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "ControllerOdomPub.inputs:timeStamp"),
                ("Odometry.outputs:position", "ControllerOdomPub.inputs:position"),
                ("Odometry.outputs:orientation", "ControllerOdomPub.inputs:orientation"),
                ("Odometry.outputs:linearVelocity", "ControllerOdomPub.inputs:linearVelocity"),
                ("Odometry.outputs:angularVelocity", "ControllerOdomPub.inputs:angularVelocity"),
                ("Odometry.outputs:execOut", "OdomTfPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "OdomTfPub.inputs:timeStamp"),
                ("Odometry.outputs:position", "OdomTfPub.inputs:translation"),
                ("Odometry.outputs:orientation", "OdomTfPub.inputs:rotation"),
                ("Tick.outputs:tick", "WorldOdomTfPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "WorldOdomTfPub.inputs:timeStamp"),
                ("Odometry.outputs:execOut", "GroundTruthTfPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "GroundTruthTfPub.inputs:timeStamp"),
                ("Odometry.outputs:position", "GroundTruthTfPub.inputs:translation"),
                ("Odometry.outputs:orientation", "GroundTruthTfPub.inputs:rotation"),
                ("Tick.outputs:tick", "JointState.inputs:execIn"),
                ("JointState.outputs:execOut", "JointPub.inputs:execIn"),
                ("JointState.outputs:jointNames", "JointPub.inputs:jointNames"),
                ("JointState.outputs:jointPositions", "JointPub.inputs:jointPositions"),
                ("JointState.outputs:jointVelocities", "JointPub.inputs:jointVelocities"),
                ("JointState.outputs:jointEfforts", "JointPub.inputs:jointEfforts"),
                ("JointState.outputs:jointDofTypes", "JointPub.inputs:jointDofTypes"),
                ("JointState.outputs:stageMetersPerUnit", "JointPub.inputs:stageMetersPerUnit"),
                ("JointState.outputs:sensorTime", "JointPub.inputs:sensorTime"),
                ("Time.outputs:simulationTime", "JointPub.inputs:timeStamp"),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    print(
        "[mobile_manipulator_sim] ROS state READY "
        "topics=/clock,/joint_states,/swerve_controller/odom,/tf,"
        "/sim/ground_truth/odom,/sim/ground_truth/tf command=/cmd_vel",
        flush=True,
    )


def add_camera_housing(
    stage: Usd.Stage,
    parent: str,
    mount_xyz: tuple[float, float, float],
    pitch: float,
    yaw: float,
) -> None:
    """Add a visible approximate D435 housing; dimensions follow the official URDF."""
    mount = UsdGeom.Xform.Define(stage, parent)
    mount.AddTranslateOp().Set(Gf.Vec3d(*mount_xyz))
    mount.AddRotateXYZOp().Set(
        Gf.Vec3f(0.0, math.degrees(pitch), math.degrees(yaw))
    )
    body = UsdGeom.Cube.Define(stage, parent + "/housing")
    body.AddTranslateOp().Set(Gf.Vec3d(0.0106, 0.0, 0.0125))
    body.AddScaleOp().Set(Gf.Vec3f(0.012525, 0.045, 0.0125))
    body.GetDisplayColorAttr().Set([Gf.Vec3f(0.18, 0.19, 0.20)])
    for index, y in enumerate((0.0325, 0.0, -0.0325)):
        lens = UsdGeom.Cylinder.Define(stage, f"{parent}/lens_{index}")
        lens.CreateAxisAttr("X")
        lens.CreateRadiusAttr(0.006)
        lens.CreateHeightAttr(0.002)
        lens.AddTranslateOp().Set(Gf.Vec3d(0.0242, y, 0.0125))
        lens.GetDisplayColorAttr().Set([Gf.Vec3f(0.02, 0.03, 0.04)])


def validate_official_openarm(stage: Usd.Stage) -> None:
    require_prim(stage, f"{OPENARM_PRIM}/openarm_body_link")
    if require_prim(stage, f"{OPENARM_PRIM}/root_joint").IsActive():
        raise RuntimeError("Official OpenArm world root joint was not disabled")
    for name in OPENARM_ARM_JOINTS:
        require_prim(stage, f"{OPENARM_PRIM}/joints/{name}")
    for side in ("left", "right"):
        require_prim(stage, f"{OPENARM_PRIM}/joints/openarm_{side}_finger_joint1")
        mimic = require_prim(
            stage, f"{OPENARM_PRIM}/joints/openarm_{side}_finger_joint2"
        )
        if not any("MimicJointAPI" in item for item in mimic.GetAppliedSchemas()):
            raise RuntimeError(f"Official {side} finger mimic schema is missing")
    require_prim(
        stage,
        f"{ROBOT_PRIM}/IntegrationJoints/base_to_official_openarm",
    )


def create_rgbd_camera(
    stage: Usd.Stage,
    name: str,
    mount_xyz: tuple[float, float, float],
    pitch: float,
    yaw: float,
) -> None:
    """Create one pinhole RGB-D sensor using the real hardware topic contract."""
    import usdrt.Sdf

    mount_prim = f"{BASE_LINK_PRIM}/{name}_mount"
    camera_prim_path = f"{mount_prim}/{name}_color"
    graph_path = f"/ProjectRgbd_{name}"
    add_camera_housing(
        stage,
        mount_prim,
        mount_xyz,
        pitch,
        yaw,
    )
    camera = UsdGeom.Camera.Define(stage, camera_prim_path)
    print(f"[mobile_manipulator_sim] {name} prim defined", flush=True)
    camera.AddTranslateOp().Set(Gf.Vec3d(*NAV_CAMERA_OPTICAL_OFFSET))
    # USD cameras look along local -Z with +Y up.  This orientation makes -Z
    # point toward base +X, matching the ROS optical-frame convention.
    camera.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.10, 10.0))
    camera.GetHorizontalApertureAttr().Set(20.955)
    if args.nav_camera_width <= 0 or args.nav_camera_height <= 0:
        raise ValueError("Navigation camera dimensions must be positive")
    if args.nav_camera_hz <= 0.0 or args.nav_camera_hz > 60.0:
        raise ValueError("Navigation camera rate must be in (0, 60] Hz")
    camera.GetVerticalApertureAttr().Set(
        20.955 * args.nav_camera_height / args.nav_camera_width
    )
    camera.GetFocalLengthAttr().Set(10.45)
    print(f"[mobile_manipulator_sim] {name} intrinsics configured", flush=True)

    camera_prim = camera.GetPrim()
    camera_prim.ApplyAPI("OmniSensorAPI")
    camera_prim.GetAttribute("omni:sensor:tickRate").Set(args.nav_camera_hz)
    print(f"[mobile_manipulator_sim] {name} sensor API configured", flush=True)

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RgbPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PointcloudPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("SemanticPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublisher", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                ("CreateRenderProduct.inputs:width", args.nav_camera_width),
                ("CreateRenderProduct.inputs:height", args.nav_camera_height),
                ("RgbPublisher.inputs:topicName", f"sensors/{name}/color/image_raw"),
                ("RgbPublisher.inputs:type", "rgb"),
                ("RgbPublisher.inputs:frameId", f"{name}_color_optical_frame"),
                ("RgbPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "DepthPublisher.inputs:topicName",
                    f"sensors/{name}/aligned_depth_to_color/image_raw",
                ),
                ("DepthPublisher.inputs:type", "depth"),
                ("DepthPublisher.inputs:frameId", f"{name}_color_optical_frame"),
                ("DepthPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "PointcloudPublisher.inputs:topicName",
                    f"sensors/{name}/depth/color/points",
                ),
                ("PointcloudPublisher.inputs:type", "depth_pcl"),
                ("PointcloudPublisher.inputs:frameId", f"{name}_color_optical_frame"),
                ("PointcloudPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "SemanticPublisher.inputs:topicName",
                    f"sensors/{name}/semantic_segmentation",
                ),
                ("SemanticPublisher.inputs:type", "semantic_segmentation"),
                ("SemanticPublisher.inputs:frameId", f"{name}_color_optical_frame"),
                # Semantic labels are only needed by the legacy single-camera
                # perception profile. Disable four redundant semantic passes in
                # the six-sensor navigation profile to fit an 8 GB laptop GPU.
                ("SemanticPublisher.inputs:enabled", not args.enable_multi_sensor_nav),
                ("SemanticPublisher.inputs:enableSemanticLabels", True),
                (
                    "SemanticPublisher.inputs:semanticLabelsTopicName",
                    f"sensors/{name}/semantic_labels",
                ),
                ("SemanticPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "CameraInfoPublisher.inputs:topicName",
                    f"sensors/{name}/color/camera_info",
                ),
                ("CameraInfoPublisher.inputs:frameId", f"{name}_color_optical_frame"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PointcloudPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "SemanticPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfoPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RgbPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PointcloudPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "SemanticPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublisher.inputs:renderProductPath"),
            ],
        },
    )
    print(f"[mobile_manipulator_sim] {name} graph created", flush=True)
    og.Controller.evaluate_sync(graph)
    print(
        f"[mobile_manipulator_sim] RGB-D READY name={name} mount={mount_xyz} "
        f"{args.nav_camera_width}x{args.nav_camera_height}@{args.nav_camera_hz:g}Hz "
        f"frame={name}_color_optical_frame",
        flush=True,
    )


def create_navigation_cameras(stage: Usd.Stage) -> None:
    specs = CAMERA_SPECS if args.enable_multi_sensor_nav else CAMERA_SPECS[:1]
    for name, mount_xyz, pitch, yaw in specs:
        create_rgbd_camera(stage, name, mount_xyz, pitch, yaw)


def create_navigation_lidars(stage: Usd.Stage) -> None:
    """Create two rotary RTX lidars and publish standard ROS LaserScan topics."""
    from isaacsim.sensors.experimental.rtx import Lidar

    keys = og.Controller.Keys
    for name, translation in LIDAR_SPECS:
        prim_path = f"{BASE_LINK_PRIM}/{name}"
        lidar = Lidar.create(
            prim_path,
            config="Example_Rotary_2D",
            tick_rate=10.0,
            translations=[translation],
        )
        graph, _, _, _ = og.Controller.edit(
            {"graph_path": f"/ProjectLidar_{name}", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("LaserScanPublisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", [lidar.paths[0]]),
                    ("LaserScanPublisher.inputs:topicName", f"sensors/{name}/scan"),
                    ("LaserScanPublisher.inputs:type", "laser_scan"),
                    ("LaserScanPublisher.inputs:frameId", f"{name}_link"),
                    ("LaserScanPublisher.inputs:resetSimulationTimeOnStop", True),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "LaserScanPublisher.inputs:execIn"),
                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        "LaserScanPublisher.inputs:renderProductPath",
                    ),
                ],
            },
        )
        og.Controller.evaluate_sync(graph)
        print(
            f"[mobile_manipulator_sim] RTX LiDAR READY name={name} "
            f"topic=/sensors/{name}/scan frame={name}_link",
            flush=True,
        )


def create_rack_unit(
    stage: Usd.Stage,
    path: str,
    center_xy: tuple[float, float],
    rack_index: int | None,
) -> None:
    """Create one freestanding three-layer planting rack."""
    root = UsdGeom.Xform.Define(stage, path)
    root.AddTranslateOp().Set(Gf.Vec3d(center_xy[0], center_xy[1], 0.0))
    color_rng = random.Random(args.rack_visual_seed * 1009 + (rack_index or 0) * 37)

    def varied_color(default: tuple[float, float, float]) -> tuple[float, float, float]:
        if args.rack_visual_seed == 0:
            return default
        return tuple(max(0.05, min(0.9, channel * color_rng.uniform(0.55, 1.6))) for channel in default)

    def label(kind: str, layer: int | None = None) -> str:
        if rack_index is None:
            return f"rack_layer_{layer}" if layer is not None else "rack_post"
        return (
            f"rack_{rack_index:02d}_layer_{layer}"
            if layer is not None
            else f"rack_{rack_index:02d}_post"
        )

    def add_box(name: str, center, size, color, semantic_label: str) -> None:
        cube = UsdGeom.Cube.Define(stage, f"{path}/{name}")
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*(value / 2.0 for value in size)))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        semantics_utils.add_labels(cube.GetPrim(), labels=semantic_label)

    # 0.80 m deep x 2.00 m long leaves useful planting area while a 1.80 m
    # clear cross-aisle and 1.40 m end aisle remain in the 4x4 layout.
    for index, height in enumerate((0.55, 1.05, 1.55), start=1):
        add_box(
            f"layer_{index}",
            (0.0, 0.0, height - 0.04),
            (0.80, 2.00, 0.08),
            varied_color((0.12, 0.48 + 0.08 * index, 0.18)),
            label("layer", index),
        )
    for x_index, x in enumerate((-0.37, 0.37)):
        for y_index, y in enumerate((-0.97, 0.97)):
            add_box(
                f"post_{x_index}_{y_index}",
                (x, y, 0.84),
                (0.06, 0.06, 1.68),
                varied_color((0.18, 0.22, 0.20)),
                label("post"),
            )


def create_parametric_validation_rack(stage: Usd.Stage) -> None:
    """Build one rack for focused perception regression."""
    create_rack_unit(stage, RACK_PRIM, (1.95, 0.0), None)
    print(
        "[mobile_manipulator_sim] parametric validation rack READY "
        "layers=3 front_x=1.55m heights=0.55,1.05,1.55m",
        flush=True,
    )


def create_rack_grid(stage: Usd.Stage) -> None:
    """Create sixteen independent racks with robot-width navigation aisles."""
    UsdGeom.Xform.Define(stage, RACK_PRIM)
    x_centers = (2.0, 4.6, 7.2, 9.8)
    y_centers = (-5.1, -1.7, 1.7, 5.1)
    rack_index = 0
    for row, x in enumerate(x_centers, start=1):
        for column, y in enumerate(y_centers, start=1):
            rack_index += 1
            create_rack_unit(
                stage,
                f"{RACK_PRIM}/Rack_{row}_{column}",
                (x, y),
                rack_index,
            )
    print(
        "[mobile_manipulator_sim] 4x4 rack grid READY racks=16 "
        "unit=0.80x2.00x1.68m cross_aisle=1.80m end_aisle=1.40m",
        flush=True,
    )


def create_cad_validation_rack(stage: Usd.Stage, asset_path: Path) -> None:
    """Load the exported full rack and add per-layer semantic proxy surfaces."""
    if not asset_path.is_file():
        raise RuntimeError(
            f"rack USD not found: {asset_path}; run tools/isaac_sim/convert_rack_stl.py"
        )

    # Source STL bounds are 8580 x 3014.2485 x 2878 mm. Rotate its long X
    # dimension onto world Y, put the near long side at world X=3.50 m and
    # center the 8.58 m length around world Y=0.
    root = UsdGeom.Xform.Define(stage, RACK_PRIM)
    root.AddTranslateOp().Set(Gf.Vec3d(6.5142485, -4.29, 0.0))
    root.AddRotateZOp().Set(90.0)
    root.AddScaleOp().Set(Gf.Vec3f(0.001, 0.001, 0.001))
    asset = stage.DefinePrim(f"{RACK_PRIM}/Asset", "Xform")
    asset.GetReferences().AddReference(str(asset_path.resolve()))
    semantics_utils.add_labels(asset, labels="rack_structure")

    def add_world_box(path: str, center, size, color, label: str) -> None:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*(value / 2.0 for value in size)))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        semantics_utils.add_labels(cube.GetPrim(), labels=label)

    # The STL is a single merged mesh, so it cannot carry independent layer
    # classes. These 30 mm overlays follow the three dominant CAD shelf bands
    # (vertex-density peaks at 0.89, 1.53 and 2.17 m). Their top faces supply
    # deterministic semantic truth while rendered depth remains colocated with
    # the real shelf structure.
    layer_heights = (0.89, 1.53, 2.17)
    for index, height in enumerate(layer_heights, start=1):
        add_world_box(
            f"{RACK_PRIM}Semantic/layer_{index}",
            (3.79, -0.29, height - 0.015),
            (0.52, 7.50, 0.03),
            (0.10, 0.42 + 0.09 * index, 0.16),
            f"rack_layer_{index}",
        )

    # Mark the two near corner uprights. The imported STL remains the primary
    # visual geometry; these narrow overlays expose a post class to perception.
    for side, y in (("left", -4.02), ("right", 4.02)):
        add_world_box(
            f"{RACK_PRIM}Semantic/post_{side}",
            (3.55, y, 1.39),
            (0.07, 0.07, 2.70),
            (0.16, 0.19, 0.17),
            "rack_post",
        )

    # A coarse invisible collision envelope protects the robot without asking
    # PhysX to cook a 254k-triangle collision mesh. RGB-D/nvblox still see the
    # detailed imported surface.
    collision = UsdGeom.Cube.Define(stage, f"{RACK_PRIM}CollisionEnvelope")
    collision.AddTranslateOp().Set(Gf.Vec3d(5.007, 0.0, 1.439))
    collision.AddScaleOp().Set(Gf.Vec3f(1.507, 4.29, 1.439))
    UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
    UsdGeom.Imageable(collision.GetPrim()).MakeInvisible()

    print(
        "[mobile_manipulator_sim] CAD validation rack READY "
        f"asset={asset_path} size=8.580x3.014x2.878m front_x=3.50m "
        "layers=0.89,1.53,2.17m",
        flush=True,
    )


def create_validation_rack(stage: Usd.Stage) -> None:
    """Create the selected rack environment."""
    layout = "single" if args.parametric_rack else args.rack_layout
    if layout == "grid4x4":
        create_rack_grid(stage)
        return
    if layout == "single":
        create_parametric_validation_rack(stage)
        return
    asset_path = Path(args.rack_asset).expanduser() if args.rack_asset else DEFAULT_RACK_USD
    create_cad_validation_rack(stage, asset_path)


def create_rack_lighting(stage: Usd.Stage) -> None:
    """Give RGB training frames a reproducible, daylight-like light source."""
    dome = UsdLux.DomeLight.Define(stage, "/World/RackDaylightDome")
    dome.CreateIntensityAttr(750.0)
    dome.CreateColorAttr(Gf.Vec3f(0.90, 0.94, 1.0))
    sun = UsdLux.DistantLight.Define(stage, "/World/RackDaylightSun")
    sun.CreateIntensityAttr(2200.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))
    sun.CreateAngleAttr(2.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 25.0, -15.0))
    print("[mobile_manipulator_sim] rack daylight READY dome=750 sun=2200", flush=True)


def nav_camera_optical_position() -> tuple[float, float, float]:
    x, y, z = NAV_CAMERA_OPTICAL_OFFSET
    cosine = math.cos(NAV_CAMERA_MOUNT_PITCH)
    sine = math.sin(NAV_CAMERA_MOUNT_PITCH)
    return (
        NAV_CAMERA_MOUNT_TRANSLATION[0] + cosine * x + sine * z,
        NAV_CAMERA_MOUNT_TRANSLATION[1] + y,
        NAV_CAMERA_MOUNT_TRANSLATION[2] - sine * x + cosine * z,
    )


def create_green_target(stage: Usd.Stage, front_depth: float) -> None:
    """Add a deterministic, known-size visual target in front of the camera."""
    if not 0.20 <= front_depth <= 3.0:
        raise ValueError("green target front depth must be in [0.20, 3.0] m")
    optical_x, optical_y, optical_z = nav_camera_optical_position()
    distance = front_depth + GREEN_TARGET_SIZE / 2.0
    center_x = optical_x + math.cos(NAV_CAMERA_MOUNT_PITCH) * distance
    # The base_link origin settles at world z=0.10 m in this stage.
    center_z = 0.10 + optical_z - math.sin(NAV_CAMERA_MOUNT_PITCH) * distance
    cube = UsdGeom.Cube.Define(stage, GREEN_TARGET_PRIM)
    cube.CreateSizeAttr(GREEN_TARGET_SIZE)
    cube.AddTranslateOp().Set(Gf.Vec3d(center_x, optical_y, center_z))

    material = UsdShade.Material.Define(stage, "/World/Looks/GreenTarget")
    shader = UsdShade.Shader.Define(stage, "/World/Looks/GreenTarget/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.02, 0.80, 0.02)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    print(
        "[mobile_manipulator_sim] green target added "
        f"center=({center_x:.4f}, 0.0000, {center_z:.4f})m "
        f"size={GREEN_TARGET_SIZE:.2f}m front_depth={front_depth:.3f}m",
        flush=True,
    )


def set_green_target_depth(stage: Usd.Stage, front_depth: float) -> None:
    """Move an existing target while keeping its center on the optical axis."""
    if not 0.20 <= front_depth <= 3.0:
        raise ValueError("green target front depth must be in [0.20, 3.0] m")
    cube = UsdGeom.Xformable(require_prim(stage, GREEN_TARGET_PRIM))
    operations = cube.GetOrderedXformOps()
    if len(operations) != 1 or operations[0].GetOpType() != UsdGeom.XformOp.TypeTranslate:
        raise RuntimeError("green target must have exactly one translate operation")
    optical_x, optical_y, optical_z = nav_camera_optical_position()
    distance = front_depth + GREEN_TARGET_SIZE / 2.0
    center_x = optical_x + math.cos(NAV_CAMERA_MOUNT_PITCH) * distance
    center_z = 0.10 + optical_z - math.sin(NAV_CAMERA_MOUNT_PITCH) * distance
    operations[0].Set(Gf.Vec3d(center_x, optical_y, center_z))
    print(
        f"[mobile_manipulator_sim] green target moved front_depth={front_depth:.3f}m",
        flush=True,
    )


def base_pose(stage: Usd.Stage) -> tuple[Gf.Vec3d, Gf.Quatd]:
    transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        require_prim(stage, BASE_LINK_PRIM)
    )
    return transform.ExtractTranslation(), transform.ExtractRotationQuat()


def roll_pitch_degrees(quaternion: Gf.Quatd) -> tuple[float, float]:
    w = float(quaternion.GetReal())
    vector = quaternion.GetImaginary()
    x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    return math.degrees(roll), math.degrees(pitch)


def create_indoor_mapping_scene(stage: Usd.Stage) -> None:
    """Create a small generic room with stable geometric and visual features."""
    colors = {
        "wall": Gf.Vec3f(0.62, 0.66, 0.70),
        "blue": Gf.Vec3f(0.08, 0.25, 0.75),
        "orange": Gf.Vec3f(0.90, 0.30, 0.05),
        "green": Gf.Vec3f(0.08, 0.65, 0.20),
        "yellow": Gf.Vec3f(0.90, 0.75, 0.05),
    }
    materials = {}
    for name, color in colors.items():
        material = UsdShade.Material.Define(stage, f"/World/Looks/Room_{name}")
        shader = UsdShade.Shader.Define(stage, f"/World/Looks/Room_{name}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.75)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        materials[name] = material

    def add_box(name, center, size, color, collision=True):
        cube = UsdGeom.Cube.Define(stage, f"/World/IndoorRoom/{name}")
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*(value / 2.0 for value in size)))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(materials[color])
        if collision:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    add_box("front_wall", (3.5, 0.0, 0.8), (0.10, 5.0, 1.6), "wall")
    add_box("rear_wall", (-3.5, 0.0, 0.8), (0.10, 5.0, 1.6), "wall")
    add_box("left_wall", (0.0, 2.5, 0.8), (7.0, 0.10, 1.6), "wall")
    add_box("right_wall", (0.0, -2.5, 0.8), (7.0, 0.10, 1.6), "wall")
    add_box("blue_crate", (1.55, 1.05, 0.35), (0.70, 0.80, 0.70), "blue")
    add_box("orange_column", (-1.45, -1.15, 0.55), (0.55, 0.55, 1.10), "orange")
    add_box("green_bench", (1.75, -1.35, 0.30), (1.10, 0.40, 0.60), "green")
    add_box("yellow_pillar", (-2.15, 1.45, 0.70), (0.28, 0.28, 1.40), "yellow")
    markers = (
        (-1.35, 0.55, "blue"),
        (-0.45, 1.05, "orange"),
        (0.55, 0.65, "green"),
        (1.45, 1.10, "yellow"),
    )
    for index, (y, z, color) in enumerate(markers):
        add_box(
            f"front_marker_{index}",
            (3.44, y, z),
            (0.025, 0.38, 0.38),
            color,
            False,
        )
    print("[mobile_manipulator_sim] indoor mapping scene READY size=7x5m", flush=True)


def create_nav_test_obstacle(stage: Usd.Stage) -> None:
    """Create hidden repeatable obstacles for RGB-D/costmap validation."""

    def add_root(obstacle_type: str) -> UsdGeom.Xform:
        root = UsdGeom.Xform.Define(stage, NAV_OBSTACLE_PRIMS[obstacle_type])
        root.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -2.0))
        root.AddRotateZOp().Set(0.0)
        UsdGeom.Imageable(root.GetPrim()).MakeInvisible()
        return root

    def add_box(root_path: str, name: str, center, size, color) -> None:
        cube = UsdGeom.Cube.Define(stage, f"{root_path}/{name}")
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*(value / 2.0 for value in size)))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    standard = add_root("standard")
    add_box(
        standard.GetPath(), "body", (0.0, 0.0, 0.40),
        (0.45, 0.45, 0.80), (0.85, 0.05, 0.05),
    )
    low = add_root("low")
    add_box(
        low.GetPath(), "body", (0.0, 0.0, 0.08),
        (0.45, 0.45, 0.16), (0.95, 0.55, 0.02),
    )
    hanging = add_root("hanging")
    add_box(
        hanging.GetPath(), "bar", (0.0, 0.0, 0.86),
        (0.40, 0.62, 0.20), (0.65, 0.10, 0.80),
    )
    shelf = add_root("shelf")
    for side in (-1.0, 1.0):
        add_box(
            shelf.GetPath(), f"post_{'left' if side > 0 else 'right'}",
            (0.0, side * 0.25, 0.50), (0.12, 0.12, 1.00),
            (0.05, 0.65, 0.75),
        )
    add_box(
        shelf.GetPath(), "deck", (0.0, 0.0, 0.62),
        (0.45, 0.62, 0.10), (0.05, 0.65, 0.75),
    )
    print(
        "[mobile_manipulator_sim] nav obstacle control READY types="
        + ",".join(NAV_OBSTACLE_PRIMS),
        flush=True,
    )


def set_nav_test_obstacle(stage: Usd.Stage, obstacle_type: str, enabled: bool) -> None:
    if obstacle_type not in NAV_OBSTACLE_PRIMS:
        raise ValueError(f"unknown navigation obstacle type: {obstacle_type}")
    root = UsdGeom.Xformable(require_prim(stage, NAV_OBSTACLE_PRIMS[obstacle_type]))
    translate_ops = [
        operation
        for operation in root.GetOrderedXformOps()
        if operation.GetOpType() == UsdGeom.XformOp.TypeTranslate
    ]
    rotate_ops = [
        operation
        for operation in root.GetOrderedXformOps()
        if operation.GetOpType() == UsdGeom.XformOp.TypeRotateZ
    ]
    if len(translate_ops) != 1:
        raise RuntimeError("navigation obstacle must have one translate operation")
    if len(rotate_ops) != 1:
        raise RuntimeError("navigation obstacle must have one rotate-Z operation")
    imageable = UsdGeom.Imageable(root.GetPrim())
    if not enabled:
        translate_ops[0].Set(Gf.Vec3d(0.0, 0.0, -2.0))
        imageable.MakeInvisible()
        print(
            f"[mobile_manipulator_sim] nav obstacle REMOVED type={obstacle_type}",
            flush=True,
        )
        return

    # Only one controlled obstacle is active at a time, which makes A/B runs
    # deterministic and prevents a previous phase contaminating the next one.
    for other_type, prim_path in NAV_OBSTACLE_PRIMS.items():
        if other_type == obstacle_type:
            continue
        other = UsdGeom.Xformable(require_prim(stage, prim_path))
        other_translate = next(
            operation for operation in other.GetOrderedXformOps()
            if operation.GetOpType() == UsdGeom.XformOp.TypeTranslate
        )
        other_translate.Set(Gf.Vec3d(0.0, 0.0, -2.0))
        UsdGeom.Imageable(other.GetPrim()).MakeInvisible()

    position, quaternion = base_pose(stage)
    w = float(quaternion.GetReal())
    qx, qy, qz = (float(value) for value in quaternion.GetImaginary())
    yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    obstacle_distance = (
        NAV_OBSTACLE_LOW_DISTANCE
        if obstacle_type == "low"
        else NAV_OBSTACLE_DISTANCE
    )
    center = Gf.Vec3d(
        float(position[0]) + obstacle_distance * math.cos(yaw),
        float(position[1]) + obstacle_distance * math.sin(yaw),
        0.0,
    )
    translate_ops[0].Set(center)
    rotate_ops[0].Set(math.degrees(yaw))
    imageable.MakeVisible()
    print(
        "[mobile_manipulator_sim] nav obstacle PLACED "
        f"type={obstacle_type} origin=({center[0]:.3f},{center[1]:.3f})m",
        flush=True,
    )


def main() -> None:
    if not ROBOT_USD.is_file():
        raise RuntimeError(f"Composed robot USD not found: {ROBOT_USD}")
    if args.realtime_factor < 0.0:
        raise ValueError("realtime factor must be non-negative")
    if args.enable_nav_obstacle_control and not args.ros:
        raise ValueError("navigation obstacle control requires --ros")

    if args.enable_multi_sensor_nav:
        args.enable_nav_rgbd = True
    ros_enabled = args.ros or args.enable_nav_rgbd
    if ros_enabled:
        print("[mobile_manipulator_sim] enabling ROS 2 bridge", flush=True)
        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        print("[mobile_manipulator_sim] ROS 2 bridge enabled", flush=True)
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
    stage = stage_utils.get_current_stage()
    if not args.empty_scene:
        create_indoor_mapping_scene(stage)
    robot = stage_utils.add_reference_to_stage(str(ROBOT_USD), ROBOT_PRIM)
    stage = robot.GetStage()
    print("[mobile_manipulator_sim] composed robot stage loaded", flush=True)
    validate_official_openarm(stage)
    configure_base_joint_drives(stage)
    # The official OpenArm asset already contains its chest D435i-B. The
    # navigation sensors are separate base-mounted sensors.
    target_depths = args.green_target_depth_sequence or [args.green_target_depth]
    if args.green_target_stage_seconds <= 0.0:
        raise ValueError("green target stage duration must be positive")
    if args.add_green_target:
        create_green_target(stage, target_depths[0])
    if args.add_rack:
        create_validation_rack(stage)
        create_rack_lighting(stage)
    if args.enable_nav_rgbd:
        create_navigation_cameras(stage)
    if args.enable_multi_sensor_nav:
        create_navigation_lidars(stage)
    if args.enable_nav_obstacle_control:
        create_nav_test_obstacle(stage)
    if ros_enabled:
        create_ros_state_graph(stage)
    ros_input = (
        RosCmdVelInput(args.enable_nav_obstacle_control) if args.ros else None
    )
    set_camera_view(
        eye=[13.0, 12.0, 11.0] if args.add_rack and args.rack_layout == "grid4x4" else [3.0, 3.0, 2.3],
        target=[5.5, 0.0, 0.7] if args.add_rack and args.rack_layout == "grid4x4" else [0.0, 0.0, 0.9],
        camera_prim_path="/OmniverseKit_Persp",
    )

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    simulation_app.update()

    initial_position, initial_orientation = base_pose(stage)
    # IsaacComputeOdometry defines odom at its first simulated sample, not at
    # graph construction. The inverted settled transform preserves world-height
    # geometry without giving odom a second parent beside RTAB-Map's map frame.
    settled_q_imag = initial_orientation.GetImaginary()
    settled_world_odom_translation, settled_world_odom_rotation = inverse_transform(
        initial_position,
        [
            float(settled_q_imag[0]),
            float(settled_q_imag[1]),
            float(settled_q_imag[2]),
            float(initial_orientation.GetReal()),
        ],
    )
    og.Controller.set(
        og.Controller.attribute(f"{ROS_GRAPH}/WorldOdomTfPub.inputs:translation"),
        settled_world_odom_translation,
    )
    og.Controller.set(
        og.Controller.attribute(f"{ROS_GRAPH}/WorldOdomTfPub.inputs:rotation"),
        settled_world_odom_rotation,
    )
    test_seconds = args.test_seconds
    if args.headless and test_seconds <= 0.0:
        test_seconds = 10.0
    max_steps = int(test_seconds * 60.0) if test_seconds > 0.0 else None
    print(
        "[mobile_manipulator_sim] READY "
        "source=NVIDIA official OpenArm bimanual; "
        f"initial_base={tuple(initial_position)}",
        flush=True,
    )

    step = 0
    wall_start = time.perf_counter()
    step_wall_period = (
        1.0 / (60.0 * args.realtime_factor)
        if args.realtime_factor > 0.0
        else 0.0
    )
    target_stage = 0
    max_abs_tilt = 0.0
    held_steering = [0.0] * 4
    try:
        while simulation_app.is_running():
            if ros_input is not None:
                vx, vy, wz = ros_input.command()
                obstacle_request = ros_input.take_obstacle_request()
                if obstacle_request is not None:
                    set_nav_test_obstacle(stage, *obstacle_request)
                if math.hypot(vx, vy) < 1.0e-4 and abs(wz) < 1.0e-4:
                    command_base(stage, held_steering, [0.0] * 4)
                else:
                    held_steering, wheel_speed = swerve_targets(vx, vy, wz)
                    # Match the real controller's stop-align-drive interlock.
                    # Rolling while the modules are still turning produces a
                    # false arc whenever manual control changes direction.
                    command_base(
                        stage,
                        held_steering,
                        wheel_speed
                        if ros_input.steering_aligned(held_steering)
                        else [0.0] * 4,
                    )
            simulation_app.update()
            step += 1
            if step_wall_period > 0.0:
                remaining = wall_start + step * step_wall_period - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)
            if args.add_green_target and len(target_depths) > 1:
                next_stage = min(
                    int((step / 60.0) / args.green_target_stage_seconds),
                    len(target_depths) - 1,
                )
                if next_stage != target_stage:
                    target_stage = next_stage
                    set_green_target_depth(stage, target_depths[target_stage])
            position, orientation = base_pose(stage)
            roll, pitch = roll_pitch_degrees(orientation)
            max_abs_tilt = max(max_abs_tilt, abs(roll), abs(pitch))
            if step % 300 == 0:
                print(
                    "[mobile_manipulator_sim] "
                    f"t={step / 60.0:.1f}s base={tuple(position)} "
                    f"roll={roll:.3f}deg pitch={pitch:.3f}deg",
                    flush=True,
                )
            if max_steps is not None and step >= max_steps:
                break
    finally:
        command_base(stage, held_steering, [0.0] * 4)
        if ros_input is not None:
            ros_input.close()
        app_utils.stop()

    final_position, final_orientation = base_pose(stage)
    final_roll, final_pitch = roll_pitch_degrees(final_orientation)
    horizontal_drift = math.hypot(
        float(final_position[0] - initial_position[0]),
        float(final_position[1] - initial_position[1]),
    )
    commanded_run = bool(
        ros_input is not None and ros_input.received_motion_command
    )
    stable = (
        all(math.isfinite(float(value)) for value in final_position)
        and (commanded_run or horizontal_drift < 0.03)
        and max_abs_tilt < 5.0
        and float(final_position[2]) > 0.02
    )
    print(
        "[mobile_manipulator_sim] RESULT "
        f"stable={stable} mode={'commanded' if commanded_run else 'stationary'} "
        f"horizontal_displacement={horizontal_drift:.6f}m "
        f"max_tilt={max_abs_tilt:.3f}deg final_z={float(final_position[2]):.4f}m "
        f"final_roll={final_roll:.3f}deg final_pitch={final_pitch:.3f}deg",
        flush=True,
    )
    if not stable:
        raise RuntimeError("Mobile manipulator failed the M1 stability limits")


failed = False
try:
    main()
except KeyboardInterrupt:
    pass
except Exception:
    # SimulationApp.close() uses a fast shutdown path that can otherwise hide
    # a pending Python traceback, making headless validation falsely look green.
    traceback.print_exc()
    failed = True
finally:
    simulation_app.close()

if failed:
    sys.exit(1)
