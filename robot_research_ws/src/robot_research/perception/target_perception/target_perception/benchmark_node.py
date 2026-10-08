"""Record synchronized detections and diagnostics against known simulation truth."""

from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticArray
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection3DArray

from target_perception.benchmark import write_artifacts


def _diagnostic_values(message: DiagnosticArray) -> dict[str, str]:
    if not message.status:
        return {}
    return {item.key: item.value for item in message.status[0].values}


class BenchmarkRecorder(Node):
    """Collect a fixed number of perception frames and then exit."""

    def __init__(self) -> None:
        super().__init__("target_perception_benchmark")
        defaults = {
            "detections_topic": "/target_perception/detections_3d",
            "metrics_topic": "/target_perception/metrics",
            "output_csv": "/tmp/target_perception_benchmark.csv",
            "sample_count": 100,
            "warmup_frames": 15,
            "expected_frame": "base_link",
            "truth_front_depth_m": 0.77,
            "truth_x_m": 1.1006,
            "truth_y_m": 0.0,
            "truth_z_m": 0.4325,
            "truth_size_x_m": 0.20,
            "truth_size_y_m": 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.output_csv = str(self.get_parameter("output_csv").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        self.warmup_frames = int(self.get_parameter("warmup_frames").value)
        if self.sample_count <= 0 or self.warmup_frames < 0:
            raise ValueError("sample_count must be positive and warmup_frames non-negative")
        self.expected_frame = str(self.get_parameter("expected_frame").value)
        self.truth = tuple(
            float(self.get_parameter(name).value)
            for name in ("truth_x_m", "truth_y_m", "truth_z_m")
        )
        self.truth_size = tuple(
            float(self.get_parameter(name).value)
            for name in ("truth_size_x_m", "truth_size_y_m")
        )
        self.front_depth = float(self.get_parameter("truth_front_depth_m").value)
        self.frames_seen = 0
        self.rows: list[dict[str, object]] = []
        self.finished = False

        detection_sub = Subscriber(
            self,
            Detection3DArray,
            str(self.get_parameter("detections_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        metrics_sub = Subscriber(
            self,
            DiagnosticArray,
            str(self.get_parameter("metrics_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = TimeSynchronizer([detection_sub, metrics_sub], queue_size=50)
        self.sync.registerCallback(self._callback)
        self.get_logger().info(
            f"benchmark waiting for {self.warmup_frames} warmup + "
            f"{self.sample_count} measured frames at depth={self.front_depth:.3f} m"
        )

    @staticmethod
    def _number(values: dict[str, str], name: str, cast=float):
        try:
            return cast(values[name])
        except (KeyError, TypeError, ValueError):
            return None

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

        values = _diagnostic_values(diagnostics)
        detected = bool(detections.detections and detections.detections[0].results)
        row: dict[str, object] = {
            "sample_index": len(self.rows),
            "stamp_sec": detections.header.stamp.sec,
            "stamp_nanosec": detections.header.stamp.nanosec,
            "detected": int(detected),
            "frame_id": detections.header.frame_id,
            "x_m": None,
            "y_m": None,
            "z_m": None,
            "error_x_m": None,
            "error_y_m": None,
            "error_z_m": None,
            "position_error_m": None,
            "size_x_m": None,
            "size_y_m": None,
            "size_error_x_m": None,
            "size_error_y_m": None,
            "confidence": self._number(values, "confidence"),
            "valid_depth_points": self._number(values, "valid_depth_points", int),
            "valid_depth_ratio": self._number(values, "valid_depth_ratio"),
        }
        if detected:
            detection = detections.detections[0]
            pose = detection.results[0].pose.pose.position
            position = (float(pose.x), float(pose.y), float(pose.z))
            error = tuple(value - truth for value, truth in zip(position, self.truth))
            size = (float(detection.bbox.size.x), float(detection.bbox.size.y))
            row.update(
                {
                    "x_m": position[0],
                    "y_m": position[1],
                    "z_m": position[2],
                    "error_x_m": error[0],
                    "error_y_m": error[1],
                    "error_z_m": error[2],
                    "position_error_m": math.sqrt(sum(item * item for item in error)),
                    "size_x_m": size[0],
                    "size_y_m": size[1],
                    "size_error_x_m": size[0] - self.truth_size[0],
                    "size_error_y_m": size[1] - self.truth_size[1],
                }
            )
            if detections.header.frame_id != self.expected_frame:
                self.get_logger().error(
                    f"unexpected detection frame {detections.header.frame_id!r}; "
                    f"expected {self.expected_frame!r}"
                )
        self.rows.append(row)
        if len(self.rows) >= self.sample_count:
            self._finish()

    def _finish(self) -> None:
        self.finished = True
        metadata = {
            "expected_frame": self.expected_frame,
            "truth_front_depth_m": self.front_depth,
            "truth_position_m": list(self.truth),
            "truth_size_m": list(self.truth_size),
            "warmup_frames": self.warmup_frames,
        }
        csv_path, summary_path, summary = write_artifacts(
            self.rows,
            self.output_csv,
            metadata,
        )
        metrics = summary["metrics"]
        rmse = metrics["position_error_rmse_m"]
        rmse_text = f"{rmse:.6f}" if rmse is not None else "unavailable"
        self.get_logger().info(
            f"BENCHMARK COMPLETE csv={csv_path} summary={summary_path} "
            f"detection_rate={metrics['detection_rate']:.3f} "
            f"rmse={rmse_text} m"
        )
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BenchmarkRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
