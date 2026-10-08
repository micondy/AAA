from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from collections import deque

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .dataset import (
    concatenate_seed_points,
    depth_points_world,
    depth_to_millimeters,
    nerfstudio_camera_to_world,
    should_capture,
    transform_matrix,
    write_binary_ply,
    write_json_atomic,
)


class DatasetRecorder(Node):
    def __init__(self) -> None:
        super().__init__("gaussian_splatting_dataset_recorder")
        self.declare_parameter("color_topic", "/sensors/nav_camera/color/image_raw")
        self.declare_parameter("depth_topic", "/sensors/nav_camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/sensors/nav_camera/color/camera_info")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("output_dir", "gaussian_dataset")
        self.declare_parameter("sync_slop_s", 0.035)
        self.declare_parameter("minimum_interval_s", 0.15)
        self.declare_parameter("minimum_translation_m", 0.03)
        self.declare_parameter("minimum_rotation_deg", 3.0)
        self.declare_parameter("integer_depth_scale_m", 0.001)
        self.declare_parameter("point_stride", 8)
        self.declare_parameter("maximum_depth_m", 5.0)
        self.declare_parameter("maximum_seed_points", 1_000_000)
        self.declare_parameter("maximum_frames", 0)
        self.declare_parameter("tf_timeout_s", 0.5)
        self.declare_parameter("pose_delay_s", 0.25)
        self.declare_parameter("pending_queue_size", 90)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser().resolve()
        self.images_dir = self.output_dir / "images"
        self.depth_dir = self.output_dir / "depth"
        if (self.output_dir / "transforms.json").exists():
            raise RuntimeError(f"refusing to overwrite existing dataset: {self.output_dir}")
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        # RGB-D odometry publishes the transform derived from the same image
        # shortly after that image arrives.  Keep TF callbacks in a separate
        # group so lookup_transform() can wait without starving the listener.
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.RLock()
        self.frames: list[dict] = []
        self.seed_chunks: list[tuple[np.ndarray, np.ndarray]] = []
        self.previous_pose: np.ndarray | None = None
        self.previous_time_s: float | None = None
        self.metadata: dict | None = None
        self.closed = False
        self.pending_frames = deque(
            maxlen=int(self.get_parameter("pending_queue_size").value)
        )

        color = message_filters.Subscriber(
            self, Image, str(self.get_parameter("color_topic").value), qos_profile=qos_profile_sensor_data
        )
        depth = message_filters.Subscriber(
            self, Image, str(self.get_parameter("depth_topic").value), qos_profile=qos_profile_sensor_data
        )
        info = message_filters.Subscriber(
            self, CameraInfo, str(self.get_parameter("camera_info_topic").value), qos_profile=qos_profile_sensor_data
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color, depth, info], queue_size=20,
            slop=float(self.get_parameter("sync_slop_s").value),
        )
        self.synchronizer.registerCallback(self._queue_frame)
        self.pending_timer = self.create_timer(0.02, self._process_pending_frame)
        self.get_logger().info(
            f"Gaussian dataset capture ready: {self.output_dir} "
            f"global_frame={self.get_parameter('global_frame').value}"
        )

    def _queue_frame(
        self, color_message: Image, depth_message: Image, info: CameraInfo
    ) -> None:
        with self.lock:
            if not self.closed:
                self.pending_frames.append(
                    (time.monotonic(), color_message, depth_message, info)
                )

    def _process_pending_frame(self) -> None:
        delay = float(self.get_parameter("pose_delay_s").value)
        with self.lock:
            if not self.pending_frames:
                return
            queued_at, color_message, depth_message, info = self.pending_frames[0]
            if time.monotonic() - queued_at < delay:
                return
            self.pending_frames.popleft()
        self._on_frame(color_message, depth_message, info)

    def _on_frame(self, color_message: Image, depth_message: Image, info: CameraInfo) -> None:
        maximum_frames = int(self.get_parameter("maximum_frames").value)
        with self.lock:
            if self.closed or (maximum_frames > 0 and len(self.frames) >= maximum_frames):
                return
        frame_id = color_message.header.frame_id or info.header.frame_id
        if not frame_id:
            self.get_logger().warn("RGB image has no frame_id", throttle_duration_sec=5.0)
            return
        stamp = Time.from_msg(color_message.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("global_frame").value), frame_id, stamp,
                timeout=Duration(
                    seconds=float(self.get_parameter("tf_timeout_s").value)
                ),
            )
        except TransformException as error:
            self.get_logger().warn(f"pose unavailable for {frame_id}: {error}", throttle_duration_sec=5.0)
            return

        ros_pose = transform_matrix(transform.transform.translation, transform.transform.rotation)
        time_s = stamp.nanoseconds * 1.0e-9
        if not should_capture(
            self.previous_pose, ros_pose, self.previous_time_s, time_s,
            float(self.get_parameter("minimum_interval_s").value),
            float(self.get_parameter("minimum_translation_m").value),
            math.radians(float(self.get_parameter("minimum_rotation_deg").value)),
        ):
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            depth_mm = depth_to_millimeters(
                raw_depth, depth_message.encoding,
                float(self.get_parameter("integer_depth_scale_m").value),
            )
        except (ValueError, TypeError) as error:
            self.get_logger().error(f"image conversion failed: {error}")
            return
        if bgr.shape[:2] != depth_mm.shape[:2]:
            self.get_logger().error(
                f"RGB/depth size mismatch: {bgr.shape[:2]} vs {depth_mm.shape[:2]}"
            )
            return

        index = len(self.frames)
        basename = f"frame_{index:06d}.png"
        if not cv2.imwrite(str(self.images_dir / basename), bgr):
            self.get_logger().error("failed to write RGB image")
            return
        if not cv2.imwrite(str(self.depth_dir / basename), depth_mm):
            self.get_logger().error("failed to write depth image")
            (self.images_dir / basename).unlink(missing_ok=True)
            return

        distortion = list(info.d) + [0.0] * 5
        if self.metadata is None:
            self.metadata = {
                "camera_model": "OPENCV",
                "fl_x": float(info.k[0]), "fl_y": float(info.k[4]),
                "cx": float(info.k[2]), "cy": float(info.k[5]),
                "w": int(info.width), "h": int(info.height),
                "k1": float(distortion[0]), "k2": float(distortion[1]),
                "p1": float(distortion[2]), "p2": float(distortion[3]),
                "depth_unit_scale_factor": 0.001,
                "global_frame": str(self.get_parameter("global_frame").value),
                "camera_frame": frame_id,
                "ply_file_path": "sparse_pc.ply",
            }
        frame = {
            "file_path": f"images/{basename}",
            "depth_file_path": f"depth/{basename}",
            "transform_matrix": nerfstudio_camera_to_world(ros_pose).tolist(),
            "timestamp_ns": int(stamp.nanoseconds),
            "time_seconds": time_s,
        }
        intrinsics = (float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5]))
        points, colors = depth_points_world(
            depth_mm, bgr, intrinsics, ros_pose,
            int(self.get_parameter("point_stride").value),
            float(self.get_parameter("maximum_depth_m").value),
        )
        with self.lock:
            self.frames.append(frame)
            self.seed_chunks.append((points, colors))
            self.previous_pose, self.previous_time_s = ros_pose, time_s
            self._write_metadata()
        if index == 0 or (index + 1) % 25 == 0:
            self.get_logger().info(f"captured {index + 1} frames")

    def _write_metadata(self) -> None:
        if self.metadata is not None:
            write_json_atomic(
                self.output_dir / "transforms.json",
                {**self.metadata, "frames": list(self.frames)},
            )

    def close(self) -> None:
        with self.lock:
            if self.closed:
                return
            self.closed = True
            points, colors = concatenate_seed_points(
                self.seed_chunks, int(self.get_parameter("maximum_seed_points").value)
            )
            if len(points):
                write_binary_ply(self.output_dir / "sparse_pc.ply", points, colors)
            self._write_metadata()
            self.get_logger().info(
                f"dataset finalized: frames={len(self.frames)} seed_points={len(points)} "
                f"path={self.output_dir}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetRecorder()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        # No synchronized-image callback may still append after the final PLY
        # and transforms.json have been written.
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
