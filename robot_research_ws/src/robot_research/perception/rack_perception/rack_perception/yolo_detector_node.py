"""Real-camera rack masks from custom-trained YOLO26n-seg weights."""

from __future__ import annotations

import json
from pathlib import Path

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rack_perception.yolo_masks import colorize_mask, encode_yolo_result, validate_rack_classes


class YoloRackDetector(Node):
    def __init__(self) -> None:
        super().__init__("rack_detector")
        defaults = {
            "weights_path": "",
            "color_topic": "/sensors/nav_camera/color/image_raw",
            "mask_topic": "/rack_detector/instance_mask",
            "debug_mask_topic": "/rack_detector/debug/color_mask",
            "metadata_topic": "/rack_detector/instances",
            "device": "auto",
            "image_size": 640,
            "confidence": 0.35,
            "iou": 0.5,
            "min_mask_pixels": 80,
            "max_rate_hz": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        raw_weights = str(self.get_parameter("weights_path").value).strip()
        if not raw_weights:
            raise RuntimeError("weights_path must point to custom-trained bed/post best.pt")
        weights = Path(raw_weights).expanduser()
        if not weights.is_file():
            raise RuntimeError(f"trained rack YOLO26n-seg weights not found: {weights}")
        try:
            from ultralytics import YOLO
            import torch
        except ImportError as error:
            raise RuntimeError("Ultralytics is not installed in this Python environment") from error
        self.model = YOLO(str(weights), task="segment")
        validate_rack_classes(self.model.names)
        device = str(self.get_parameter("device").value)
        self.device = ("0" if torch.cuda.is_available() else "cpu") if device == "auto" else device
        self.bridge = CvBridge()
        self.pending: Image | None = None
        self.mask_pub = self.create_publisher(
            Image, str(self.get_parameter("mask_topic").value), qos_profile_sensor_data
        )
        self.debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_mask_topic").value), qos_profile_sensor_data
        )
        self.metadata_pub = self.create_publisher(
            String, str(self.get_parameter("metadata_topic").value), 10
        )
        self.create_subscription(
            Image, str(self.get_parameter("color_topic").value), self._on_color,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / max(float(self.get_parameter("max_rate_hz").value), 0.1), self._infer)
        self.get_logger().info(f"rack YOLO26n-seg ready: {weights}, device={self.device}")

    def _on_color(self, message: Image) -> None:
        # Keep only the newest frame when inference is slower than the camera.
        self.pending = message

    def _infer(self) -> None:
        message, self.pending = self.pending, None
        if message is None:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = self.model.predict(
                source=bgr,
                imgsz=int(self.get_parameter("image_size").value),
                conf=float(self.get_parameter("confidence").value),
                iou=float(self.get_parameter("iou").value),
                device=self.device,
                retina_masks=True,
                verbose=False,
            )[0]
            mask, instances = encode_yolo_result(
                result, bgr.shape[:2], int(self.get_parameter("min_mask_pixels").value)
            )
        except (CvBridgeError, RuntimeError, ValueError) as error:
            self.get_logger().warning(f"rack YOLO frame skipped: {error}", throttle_duration_sec=2.0)
            return
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono16")
        mask_msg.header = message.header
        self.mask_pub.publish(mask_msg)
        debug_msg = self.bridge.cv2_to_imgmsg(colorize_mask(mask), encoding="bgr8")
        debug_msg.header = message.header
        self.debug_pub.publish(debug_msg)
        metadata = String()
        metadata.data = json.dumps(
            {"schema": "rack_masks/v1", "instances": instances}, separators=(",", ":")
        )
        self.metadata_pub.publish(metadata)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloRackDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
