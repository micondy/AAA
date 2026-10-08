#!/usr/bin/env python3
"""Run the official Nova Carter ROS 2 warehouse scene for project validation.

This script must be launched with Isaac Sim's ``python.sh``.  It intentionally
does not publish velocity commands: external ROS 2 nodes remain the only motion
command authority during validation.
"""

from __future__ import annotations

import argparse

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--enable-front-rgb",
        action="store_true",
        help="Enable the front Hawk left/right RGB render products.",
    )
    parser.add_argument(
        "--enable-nav-rgbd",
        action="store_true",
        help="Publish the front-left camera as project RGB, depth, info and point cloud topics.",
    )
    args, _ = parser.parse_known_args()
    return args


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni
import omni.graph.core as og
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.experimental.utils.stage import is_stage_loading
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom


SCENE_PATH = "/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
FRONT_CAMERA_GRAPH = "/World/Nova_Carter_ROS/front_hawk"
FRONT_LEFT_CAMERA = (
    "/World/Nova_Carter_ROS/chassis_link/sensors/front_hawk/left/camera_left"
)
PROJECT_CAMERA_GRAPH = "/ProjectNavCamera"
FRONT_3D_LIDAR_RENDER_PRODUCT = (
    "/World/Nova_Carter_ROS/ros_lidars/front_3d_lidar_render_product.inputs:enabled"
)
GROUND_TRUTH_ODOM_TOPIC = (
    "/World/Nova_Carter_ROS/transform_tree_odometry/"
    "ros2_publish_odometry.inputs:topicName"
)
GROUND_TRUTH_TF_TOPIC = (
    "/World/Nova_Carter_ROS/transform_tree_odometry/"
    "ros2_publish_raw_transform_tree.inputs:topicName"
)
SENSOR_TF_TOPIC = (
    "/World/Nova_Carter_ROS/transform_tree_odometry/"
    "tf_tree_base_link_to_sensors.inputs:topicName"
)


def set_graph_enabled(attribute_path: str, enabled: bool) -> None:
    attribute = og.Controller.attribute(attribute_path)
    if attribute.is_valid():
        og.Controller.set(attribute, enabled)
    else:
        carb.log_warn(f"Optional graph attribute not found: {attribute_path}")


def set_graph_value(attribute_path: str, value: object) -> None:
    attribute = og.Controller.attribute(attribute_path)
    if not attribute.is_valid():
        raise RuntimeError(f"Required graph attribute not found: {attribute_path}")
    og.Controller.set(attribute, value)


def print_front_camera_graph() -> None:
    """Print the camera graph contract supplied by the referenced USD asset."""
    stage = omni.usd.get_context().get_stage()
    graph_prim = stage.GetPrimAtPath(FRONT_CAMERA_GRAPH)
    if not graph_prim.IsValid():
        carb.log_warn(f"Front camera graph not found: {FRONT_CAMERA_GRAPH}")
        return

    print("[isaac_sim_bringup] Front camera graph:", flush=True)
    for prim in Usd.PrimRange(graph_prim):
        values = []
        for attribute in prim.GetAttributes():
            name = attribute.GetName()
            if name in {"inputs:enabled", "inputs:topicName", "inputs:type"}:
                values.append(f"{name}={attribute.Get()!r}")
        if values:
            print(
                f"  {prim.GetPath()} ({prim.GetTypeName()}): " + ", ".join(values),
                flush=True,
            )


def create_project_nav_camera_graph() -> None:
    """Publish one simulated camera with the same contract as nav_camera."""
    import usdrt.Sdf

    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(FRONT_LEFT_CAMERA)
    if not camera_prim.IsValid():
        raise RuntimeError(f"Camera prim not found: {FRONT_LEFT_CAMERA}")

    camera_prim.ApplyAPI("OmniSensorAPI")
    camera_prim.GetAttribute("omni:sensor:tickRate").Set(15.0)
    projection_attribute = camera_prim.GetAttribute("cameraProjectionType")
    if projection_attribute.IsValid():
        original_projection = projection_attribute.Get()
        projection_attribute.Set("pinhole")
        print(
            "[isaac_sim_bringup] Camera projection overridden: "
            f"{original_projection!r} -> 'pinhole'",
            flush=True,
        )
    camera = UsdGeom.Camera(camera_prim)
    horizontal_aperture = camera.GetHorizontalApertureAttr().Get()
    camera.GetVerticalApertureAttr().Set(horizontal_aperture * 480.0 / 640.0)

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": PROJECT_CAMERA_GRAPH, "evaluator_name": "execution"},
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
                (
                    "CreateRenderProduct.inputs:cameraPrim",
                    [usdrt.Sdf.Path(FRONT_LEFT_CAMERA)],
                ),
                ("CreateRenderProduct.inputs:width", 640),
                ("CreateRenderProduct.inputs:height", 480),
                ("RgbPublisher.inputs:topicName", "sensors/nav_camera/color/image_raw"),
                ("RgbPublisher.inputs:type", "rgb"),
                ("RgbPublisher.inputs:frameId", "front_stereo_camera_left_rgb"),
                ("RgbPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "DepthPublisher.inputs:topicName",
                    "sensors/nav_camera/aligned_depth_to_color/image_raw",
                ),
                ("DepthPublisher.inputs:type", "depth"),
                ("DepthPublisher.inputs:frameId", "front_stereo_camera_left_rgb"),
                ("DepthPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "PointcloudPublisher.inputs:topicName",
                    "sensors/nav_camera/depth/color/points",
                ),
                ("PointcloudPublisher.inputs:type", "depth_pcl"),
                ("PointcloudPublisher.inputs:frameId", "front_stereo_camera_left_rgb"),
                ("PointcloudPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "CameraInfoPublisher.inputs:topicName",
                    "sensors/nav_camera/color/camera_info",
                ),
                ("CameraInfoPublisher.inputs:frameId", "front_stereo_camera_left_rgb"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PointcloudPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfoPublisher.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "RgbPublisher.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "DepthPublisher.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PointcloudPublisher.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "CameraInfoPublisher.inputs:renderProductPath",
                ),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    print(
        "[isaac_sim_bringup] Project nav RGB-D enabled at 640x480, 15 Hz",
        flush=True,
    )


def main() -> None:
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Isaac Sim assets root is unavailable")

    scene_url = assets_root + SCENE_PATH
    print(f"[isaac_sim_bringup] Loading {scene_url}", flush=True)
    omni.usd.get_context().open_stage(scene_url, None)
    simulation_app.update()
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    print_front_camera_graph()

    # The official scene keeps these render products disabled until requested.
    # S0 needs no camera; S1 can opt into only the front stereo pair.
    set_graph_enabled(
        FRONT_CAMERA_GRAPH + "/left_camera_render_product.inputs:enabled",
        args.enable_front_rgb,
    )
    set_graph_enabled(
        FRONT_CAMERA_GRAPH + "/right_camera_render_product.inputs:enabled",
        args.enable_front_rgb,
    )
    set_graph_enabled(FRONT_3D_LIDAR_RENDER_PRODUCT, False)

    # Preserve physical odometry as evaluation-only ground truth.  Its TF is
    # moved away from /tf so RTAB-Map can be the sole odom -> base_link owner.
    set_graph_value(GROUND_TRUTH_ODOM_TOPIC, "sim/ground_truth/odom")
    set_graph_value(GROUND_TRUTH_TF_TOPIC, "sim/ground_truth/tf")
    # The camera is rigidly mounted, but the official asset publishes its
    # transform dynamically.  Rendering can lead that dynamic TF by a frame,
    # so Carter ROS launch profiles provide the verified mount on /tf_static.
    set_graph_value(SENSOR_TF_TOPIC, "sim/ground_truth/sensor_tf")
    print(
        "[isaac_sim_bringup] Ground truth remapped to "
        "/sim/ground_truth/odom, /sim/ground_truth/tf and "
        "/sim/ground_truth/sensor_tf",
        flush=True,
    )
    if args.enable_nav_rgbd:
        create_project_nav_camera_graph()

    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/publish_without_verification", True
    )
    app_utils.play()
    simulation_app.update()
    for _ in range(60):
        simulation_app.update()

    print(
        "[isaac_sim_bringup] READY: simulation is playing; "
        "send ROS 2 Twist commands on /cmd_vel",
        flush=True,
    )
    while simulation_app.is_running():
        simulation_app.update()


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    app_utils.stop()
    simulation_app.close()
