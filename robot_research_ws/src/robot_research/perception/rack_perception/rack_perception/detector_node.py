"""Adapt Isaac semantic labels into a stable compact rack mask."""

from __future__ import annotations

import json
import random
import re

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def anonymous_mask_ids(
    visible: list[tuple[int, int, str, int]], rng: random.Random
) -> dict[int, int]:
    """Give every visible bed/post a fresh frame-local ID without truth numbering."""
    beds = [source_id for source_id, encoded_id, _, _ in visible if encoded_id < 10000]
    posts = [source_id for source_id, encoded_id, _, _ in visible if encoded_id >= 10000]
    mapping = dict(zip(beds, rng.sample(range(1, 10000), len(beds))))
    mapping.update(zip(posts, rng.sample(range(10000, 65536), len(posts))))
    return mapping


class RackDetector(Node):
    """Simulation-label frontend; replace this node with learned segmentation later."""

    def __init__(self) -> None:
        super().__init__("rack_detector")
        defaults = {
            "semantic_image_topic": "/sensors/nav_camera/semantic_segmentation",
            "semantic_labels_topic": "/sensors/nav_camera/semantic_labels",
            "mask_topic": "/rack_detector/instance_mask",
            "debug_mask_topic": "/rack_detector/debug/color_mask",
            "metadata_topic": "/rack_detector/instances",
            "layer_label_pattern": r"rack_layer_(\d+)",
            "post_label": "rack_post",
            "anonymous_instances": False,
            "anonymous_seed": 2026,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.layer_pattern = re.compile(
            str(self.get_parameter("layer_label_pattern").value), re.IGNORECASE
        )
        self.instance_layer_pattern = re.compile(
            r"rack_(\d+)_layer_(\d+)", re.IGNORECASE
        )
        self.instance_post_pattern = re.compile(r"rack_(\d+)_post", re.IGNORECASE)
        self.post_label = str(self.get_parameter("post_label").value).lower()
        self.anonymous_instances = bool(self.get_parameter("anonymous_instances").value)
        self.anonymous_rng = random.Random(int(self.get_parameter("anonymous_seed").value))
        self.id_map: dict[int, tuple[int, str]] = {}
        self.mask_pub = self.create_publisher(
            Image, str(self.get_parameter("mask_topic").value), qos_profile_sensor_data
        )
        self.debug_mask_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_mask_topic").value),
            qos_profile_sensor_data,
        )
        self.metadata_pub = self.create_publisher(
            String, str(self.get_parameter("metadata_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantic_labels_topic").value),
            self._labels_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("semantic_image_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("Isaac semantic rack detector ready")

    def _labels_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            mapping: dict[int, tuple[int, str]] = {}
            for raw_id, info in payload.items():
                if raw_id == "time_stamp" or not isinstance(info, dict):
                    continue
                label = str(info.get("class", "")).lower()
                instance_match = self.instance_layer_pattern.fullmatch(label)
                match = self.layer_pattern.fullmatch(label)
                post_match = self.instance_post_pattern.fullmatch(label)
                if instance_match:
                    rack_number = int(instance_match.group(1))
                    layer_number = int(instance_match.group(2))
                    mapping[int(raw_id)] = (rack_number * 10 + layer_number, label)
                elif match:
                    layer_number = int(match.group(1))
                    mapping[int(raw_id)] = (layer_number, label)
                elif post_match:
                    mapping[int(raw_id)] = (10000 + int(post_match.group(1)), label)
                elif label == self.post_label:
                    mapping[int(raw_id)] = (10000, label)
            self.id_map = mapping
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"invalid Isaac semantic-label dictionary: {error}")

    def _image_callback(self, message: Image) -> None:
        if not self.id_map:
            return
        try:
            semantic = np.asarray(self.bridge.imgmsg_to_cv2(message, "passthrough"))
        except CvBridgeError as error:
            self.get_logger().warning(f"semantic image conversion failed: {error}")
            return
        if semantic.ndim == 3:
            semantic = semantic[..., 0]
        output = np.zeros(semantic.shape, dtype=np.uint16)
        instances = []
        visible = []
        for source_id, (encoded_id, label) in self.id_map.items():
            count = int(np.count_nonzero(semantic == source_id))
            if count:
                visible.append((source_id, encoded_id, label, count))
        temporary_ids = anonymous_mask_ids(visible, self.anonymous_rng) if self.anonymous_instances else {}
        for source_id, encoded_id, label, count in visible:
            output_id = temporary_ids[source_id] if self.anonymous_instances else encoded_id
            output[semantic == source_id] = output_id
            instances.append({
                "mask_id": output_id,
                "label": "post" if encoded_id >= 10000 else "bed",
                "pixels": count,
            } if self.anonymous_instances else {
                "mask_id": output_id, "semantic_id": source_id, "label": label, "pixels": count,
            })
        mask_message = self.bridge.cv2_to_imgmsg(output, encoding="mono16")
        mask_message.header = message.header
        self.mask_pub.publish(mask_message)

        # Keep the numeric mono16 mask as the machine interface, and publish a
        # separate BGR visualization so low label values do not appear black.
        color = np.zeros((*output.shape, 3), dtype=np.uint8)
        layer_number = output % 3 if self.anonymous_instances else output % 10
        color[(output > 0) & (output < 10000) & (layer_number == (0 if self.anonymous_instances else 1))] = (40, 210, 40)
        color[(output > 0) & (output < 10000) & (layer_number == (1 if self.anonymous_instances else 2))] = (30, 180, 255)
        color[(output > 0) & (output < 10000) & (layer_number == (2 if self.anonymous_instances else 3))] = (220, 80, 60)
        color[output >= 10000] = (220, 220, 220)
        debug_message = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        debug_message.header = message.header
        self.debug_mask_pub.publish(debug_message)

        metadata = String()
        metadata.data = json.dumps(
            {"schema": "rack_masks/v1", "instances": instances}, separators=(",", ":")
        )
        self.metadata_pub.publish(metadata)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RackDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
