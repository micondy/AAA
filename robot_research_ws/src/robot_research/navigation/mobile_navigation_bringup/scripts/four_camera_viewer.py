#!/usr/bin/env python3
"""Local four-camera RGB viewer for the Jazzy simulation profile."""

from __future__ import annotations

import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


CAMERAS = ("nav_camera", "rear_camera", "left_camera", "right_camera")
MAX_FRAME_AGE_SECONDS = 3.0


class FourCameraViewer(Node):
    def __init__(self) -> None:
        super().__init__("four_camera_viewer")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8090)
        self.declare_parameter("max_jpeg_width", 640)
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.max_jpeg_width = int(self.get_parameter("max_jpeg_width").value)
        if not 0 < self.port < 65536 or self.max_jpeg_width < 1:
            raise ValueError("port must be 1..65535 and max_jpeg_width must be positive")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames: dict[str, tuple[bytes, float]] = {}
        self._image_subscriptions = []
        for name in CAMERAS:
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    f"/sensors/{name}/color/image_raw",
                    lambda message, camera=name: self.on_image(camera, message),
                    qos_profile_sensor_data,
                )
            )

    def on_image(self, name: str, message: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            height, width = bgr.shape[:2]
            if width > self.max_jpeg_width:
                scale = self.max_jpeg_width / width
                bgr = cv2.resize(
                    bgr,
                    (self.max_jpeg_width, round(height * scale)),
                    interpolation=cv2.INTER_AREA,
                )
            ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
        except (RuntimeError, ValueError, cv2.error) as error:
            self.get_logger().warning(f"Could not encode {name}: {error}")
            return
        with self.lock:
            self.frames[name] = (encoded.tobytes(), time.monotonic())

    def snapshot(self, name: str) -> bytes | None:
        with self.lock:
            frame = self.frames.get(name)
        if frame is None or time.monotonic() - frame[1] > MAX_FRAME_AGE_SECONDS:
            return None
        return frame[0]

    def status(self) -> dict[str, bool]:
        now = time.monotonic()
        with self.lock:
            return {
                name: name in self.frames
                and now - self.frames[name][1] <= MAX_FRAME_AGE_SECONDS
                for name in CAMERAS
            }


def make_handler(viewer: FourCameraViewer, page: bytes):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                self.respond(200, "text/html; charset=utf-8", page)
            elif path == "/api/status":
                body = json.dumps(viewer.status()).encode("utf-8")
                self.respond(200, "application/json; charset=utf-8", body)
            elif path.startswith("/api/camera/") and path.endswith(".jpg"):
                name = path.removeprefix("/api/camera/").removesuffix(".jpg")
                if name not in CAMERAS:
                    self.respond(404, "text/plain; charset=utf-8", b"Unknown camera")
                    return
                image = viewer.snapshot(name)
                if image is None:
                    self.respond(503, "text/plain; charset=utf-8", b"Camera offline")
                else:
                    self.respond(200, "image/jpeg", image)
            else:
                self.respond(404, "text/plain; charset=utf-8", b"Not found")

        def respond(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format_string: str, *args: object) -> None:
            return

    return Handler


def main() -> None:
    rclpy.init()
    viewer = FourCameraViewer()
    page = (
        Path(get_package_share_directory("mobile_navigation_bringup"))
        / "web"
        / "four_cameras.html"
    ).read_bytes()
    server = ThreadingHTTPServer((viewer.host, viewer.port), make_handler(viewer, page))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    viewer.get_logger().info(f"Four-camera viewer: http://{viewer.host}:{viewer.port}/")
    try:
        rclpy.spin(viewer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch forwards another SIGINT during child shutdown. Let the
        # first one stop spinning, then finish HTTP and ROS cleanup unhindered.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        server.shutdown()
        server.server_close()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
