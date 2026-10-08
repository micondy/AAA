"""Record a fixed multi-object perception run without requiring ground truth."""

from __future__ import annotations

from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection3DArray

from target_perception.general_object_benchmark import (
    write_general_object_artifacts,
)


def _diagnostic(message: DiagnosticArray) -> tuple[int, str, dict[str, str]]:
    if not message.status:
        return -1, "missing diagnostics", {}
    status = message.status[0]
    level = status.level
    if isinstance(level, (bytes, bytearray)):
        level = int.from_bytes(level, byteorder="little") if level else 0
    return (
        int(level),
        status.message,
        {item.key: item.value for item in status.values},
    )


def _number(values: dict[str, str], key: str, cast=float):
    try:
        return cast(values[key])
    except (KeyError, TypeError, ValueError):
        return None


class GeneralObjectBenchmarkRecorder(Node):
    """Collect synchronized multi-instance detections and runtime diagnostics."""

    def __init__(self) -> None:
        super().__init__("general_object_benchmark")
        defaults = {
            "detections_topic": "/target_perception/detections_3d",
            "metrics_topic": "/target_perception/metrics",
            "output_dir": "/tmp/general_object_benchmark",
            "sample_count": 100,
            "warmup_frames": 15,
            "expected_frame": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.output_dir = str(self.get_parameter("output_dir").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        self.warmup_frames = int(self.get_parameter("warmup_frames").value)
        self.expected_frame = str(self.get_parameter("expected_frame").value)
        if self.sample_count <= 0 or self.warmup_frames < 0:
            raise ValueError("sample_count must be positive and warmup_frames non-negative")
        output_path = Path(self.output_dir).expanduser()
        if output_path.exists() and any(output_path.iterdir()):
            raise FileExistsError(
                f"benchmark output directory is not empty: {output_path.resolve()}"
            )
        self.frames_seen = 0
        self.frame_rows: list[dict[str, object]] = []
        self.instance_rows: list[dict[str, object]] = []
        self.finished = False

        detections = Subscriber(
            self,
            Detection3DArray,
            str(self.get_parameter("detections_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        metrics = Subscriber(
            self,
            DiagnosticArray,
            str(self.get_parameter("metrics_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = TimeSynchronizer([detections, metrics], queue_size=50)
        self.sync.registerCallback(self._callback)
        self.get_logger().info(
            f"general-object benchmark waiting for {self.warmup_frames} warmup + "
            f"{self.sample_count} measured frames"
        )

    def _callback(
        self,
        detections: Detection3DArray,
        diagnostics: DiagnosticArray,
    ) -> None:
        if self.finished:
            return
        self.frames_seen += 1
        if self.frames_seen <= self.warmup_frames:
            return

        frame_index = len(self.frame_rows)
        level, message, values = _diagnostic(diagnostics)
        valid_detections = [item for item in detections.detections if item.results]
        model_count = _number(values, "model_instances", int)
        depth_count = _number(values, "depth_valid_instances", int)
        frame_row = {
            "frame_index": frame_index,
            "stamp_sec": detections.header.stamp.sec,
            "stamp_nanosec": detections.header.stamp.nanosec,
            "frame_id": detections.header.frame_id,
            "detection_count": len(valid_detections),
            "model_instance_count": model_count,
            "depth_valid_instance_count": depth_count,
            "depth_valid_ratio": (
                depth_count / model_count if model_count else None
            ),
            "inference_and_fusion_ms": _number(
                values, "inference_and_fusion_ms"
            ),
            "device": values.get("device", ""),
            "weights_sha256": values.get("weights_sha256", ""),
            "diagnostic_level": level,
            "diagnostic_message": message,
        }
        self.frame_rows.append(frame_row)

        if self.expected_frame and detections.header.frame_id != self.expected_frame:
            self.get_logger().error(
                f"unexpected frame {detections.header.frame_id!r}; "
                f"expected {self.expected_frame!r}"
            )
        for instance_index, detection in enumerate(valid_detections):
            result = detection.results[0]
            position = result.pose.pose.position
            covariance = result.pose.covariance
            self.instance_rows.append(
                {
                    "frame_index": frame_index,
                    "instance_index": instance_index,
                    "detection_id": detection.id,
                    "class_id": result.hypothesis.class_id,
                    "confidence": float(result.hypothesis.score),
                    "x_m": float(position.x),
                    "y_m": float(position.y),
                    "z_m": float(position.z),
                    "size_x_m": float(detection.bbox.size.x),
                    "size_y_m": float(detection.bbox.size.y),
                    "size_z_m": float(detection.bbox.size.z),
                    "position_covariance_trace_m2": float(
                        covariance[0] + covariance[7] + covariance[14]
                    ),
                }
            )
        if len(self.frame_rows) >= self.sample_count:
            self._finish(request_shutdown=True)

    def _finish(self, request_shutdown: bool) -> None:
        self.finished = True
        devices = sorted({str(row["device"]) for row in self.frame_rows if row["device"]})
        hashes = sorted(
            {
                str(row["weights_sha256"])
                for row in self.frame_rows
                if row["weights_sha256"]
            }
        )
        frame_path, instance_path, summary_path, artifact = (
            write_general_object_artifacts(
                self.frame_rows,
                self.instance_rows,
                self.output_dir,
                {
                    "sample_count_requested": self.sample_count,
                    "warmup_frames": self.warmup_frames,
                    "expected_frame": self.expected_frame or None,
                    "devices": devices,
                    "weights_sha256": hashes,
                },
            )
        )
        metrics = artifact["metrics"]
        self.get_logger().info(
            f"GENERAL OBJECT BENCHMARK COMPLETE frames={frame_path} "
            f"instances={instance_path} summary={summary_path} "
            f"detection_rate={metrics['frame_detection_rate']:.3f}"
        )
        if request_shutdown:
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GeneralObjectBenchmarkRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.finished and node.frame_rows:
            node._finish(request_shutdown=False)
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
