#!/usr/bin/env python3
"""Observe whether trained YOLO masks flow through rack_geometry and rack_map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Observer(Node):
    def __init__(self) -> None:
        super().__init__("rack_yolo_online_check")
        self.masks = 0
        self.metadata_frames = 0
        self.bed_instances = 0
        self.post_instances = 0
        self.geometry_frames = 0
        self.geometry_layers = 0
        self.map_frames = 0
        self.maximum_racks = 0
        self.maximum_layers = 0
        self.create_subscription(Image, "/rack_detector/instance_mask", self._mask, qos_profile_sensor_data)
        self.create_subscription(String, "/rack_detector/instances", self._metadata, 10)
        self.create_subscription(String, "/rack_geometry/observations", self._geometry, 10)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/rack_map/model", self._map, latched)

    def _mask(self, _: Image) -> None:
        self.masks += 1

    def _metadata(self, message: String) -> None:
        data = json.loads(message.data)
        self.metadata_frames += 1
        self.bed_instances += sum(item["label"] == "bed" for item in data["instances"])
        self.post_instances += sum(item["label"] == "post" for item in data["instances"])

    def _geometry(self, message: String) -> None:
        data = json.loads(message.data)
        self.geometry_frames += 1
        self.geometry_layers += len(data.get("layers", data.get("observations", [])))

    def _map(self, message: String) -> None:
        data = json.loads(message.data)
        self.map_frames += 1
        racks = data.get("racks", [])
        self.maximum_racks = max(self.maximum_racks, len(racks))
        self.maximum_layers = max(self.maximum_layers, sum(len(rack["layers"]) for rack in racks))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Observer()
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - start
    report = {
        "schema": "rack_yolo_online_check/v1",
        "elapsed_wall_s": elapsed,
        "mask_frames": node.masks,
        "mask_hz": node.masks / elapsed if elapsed else 0.0,
        "metadata_frames": node.metadata_frames,
        "bed_instances": node.bed_instances,
        "post_instances": node.post_instances,
        "geometry_frames": node.geometry_frames,
        "geometry_layers": node.geometry_layers,
        "map_frames": node.map_frames,
        "maximum_racks": node.maximum_racks,
        "maximum_layers": node.maximum_layers,
    }
    report["passed"] = (
        report["mask_frames"] > 0 and report["bed_instances"] > 0
        and report["geometry_layers"] > 0 and report["maximum_layers"] > 0
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
