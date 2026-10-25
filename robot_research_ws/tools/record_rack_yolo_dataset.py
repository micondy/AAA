#!/usr/bin/env python3
"""Record paired Isaac RGB/instance-mask frames as a YOLO segmentation split."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from cv_bridge import CvBridge, CvBridgeError
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rack_perception.yolo_dataset import mask_to_yolo_rows


class RackDatasetRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rack_yolo_dataset_recorder")
        self.args = args
        self.bridge = CvBridge()
        self.images = args.output / "images" / args.split
        self.labels = args.output / "labels" / args.split
        if self.images.exists() and any(self.images.glob("frame_*.png")):
            raise RuntimeError(f"split already contains frames; refusing to overwrite: {self.images}")
        self.images.mkdir(parents=True, exist_ok=True)
        self.labels.mkdir(parents=True, exist_ok=True)
        self.last_stamp_s = float("-inf")
        self.frames = 0
        self.dark_rejections = 0
        self.counts = {"bed": 0, "post": 0}
        rgb = Subscriber(self, Image, args.rgb_topic, qos_profile=qos_profile_sensor_data)
        mask = Subscriber(self, Image, args.mask_topic, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb, mask], queue_size=90, slop=args.slop_s)
        self.sync.registerCallback(self._on_pair)

    def _on_pair(self, rgb_msg: Image, mask_msg: Image) -> None:
        if self.frames >= self.args.max_frames:
            return
        stamp_s = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        if stamp_s - self.last_stamp_s < self.args.interval_s:
            return
        try:
            bgr = np.asarray(self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8"))
            mask = np.asarray(self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono16"))
            if bgr.shape[:2] != mask.shape:
                raise ValueError("RGB and mask dimensions differ")
            luminance = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            if (float(np.median(luminance)) < self.args.minimum_median_luma
                    or float(np.mean(luminance < 12)) > self.args.maximum_black_fraction):
                self.dark_rejections += 1
                return
            rows, counts = mask_to_yolo_rows(mask)
        except (CvBridgeError, ValueError) as error:
            self.get_logger().warning(f"dataset frame skipped: {error}", throttle_duration_sec=2.0)
            return
        if counts["bed"] < self.args.minimum_beds:
            return
        basename = f"frame_{self.frames:06d}"
        image_path = self.images / (basename + ".png")
        if not cv2.imwrite(str(image_path), bgr):
            raise RuntimeError(f"failed to save {image_path}")
        (self.labels / (basename + ".txt")).write_text("\n".join(rows) + "\n", encoding="utf-8")
        self.frames += 1
        self.last_stamp_s = stamp_s
        for category in counts:
            self.counts[category] += counts[category]
        if self.frames % 25 == 0:
            self.get_logger().info(f"captured {self.frames}/{self.args.max_frames} {self.args.split} frames")

    def write_manifest(self) -> Path:
        manifest = self.args.output / f"capture_{self.args.split}.json"
        manifest.write_text(json.dumps({
            "schema": "rack_yolo_capture/v1",
            "split": self.args.split,
            "frames": self.frames,
            "instances": self.counts,
            "rgb_topic": self.args.rgb_topic,
            "mask_topic": self.args.mask_topic,
            "source": "Isaac semantic labels through rack_detector mono16",
            "visual_seed": self.args.visual_seed,
            "interval_s": self.args.interval_s,
            "minimum_beds": self.args.minimum_beds,
            "minimum_median_luma": self.args.minimum_median_luma,
            "maximum_black_fraction": self.args.maximum_black_fraction,
            "dark_rejections": self.dark_rejections,
        }, indent=2), encoding="utf-8")
        return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "val"), required=True)
    parser.add_argument("--visual-seed", type=int, required=True)
    parser.add_argument("--rgb-topic", default="/sensors/nav_camera/color/image_raw")
    parser.add_argument("--mask-topic", default="/rack_detector/instance_mask")
    parser.add_argument("--interval-s", type=float, default=0.5)
    parser.add_argument("--slop-s", type=float, default=0.08)
    parser.add_argument("--minimum-beds", type=int, default=1)
    parser.add_argument("--minimum-median-luma", type=float, default=35.0)
    parser.add_argument("--maximum-black-fraction", type=float, default=0.20)
    parser.add_argument("--max-frames", type=int, default=300)
    parser.add_argument("--duration-s", type=float, default=300.0)
    args = parser.parse_args()
    if args.max_frames <= 0 or args.duration_s <= 0 or args.interval_s <= 0:
        parser.error("max-frames, duration-s, and interval-s must be positive")
    if not 0 <= args.minimum_median_luma <= 255 or not 0 <= args.maximum_black_fraction <= 1:
        parser.error("brightness thresholds must be within image range")
    rclpy.init()
    node = RackDatasetRecorder(args)
    start = time.monotonic()
    try:
        while rclpy.ok() and node.frames < args.max_frames and time.monotonic() - start < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        manifest = node.write_manifest()
        print(f"[rack_yolo_dataset] frames={node.frames} instances={node.counts} manifest={manifest}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
