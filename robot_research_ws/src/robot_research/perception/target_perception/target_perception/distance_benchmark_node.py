"""Collect a multi-distance benchmark while Isaac moves one known target."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection3DArray

from target_perception.benchmark import write_artifacts


SUMMARY_FIELDS = (
    "truth_front_depth_m",
    "samples",
    "detections",
    "failures",
    "detection_rate",
    "position_error_mean_m",
    "position_error_std_m",
    "position_error_rmse_m",
    "position_error_max_m",
    "bias_x_m",
    "bias_y_m",
    "bias_z_m",
    "size_error_x_mean_m",
    "size_error_y_mean_m",
    "valid_depth_ratio_mean",
    "confidence_mean",
)


def _values(message: DiagnosticArray) -> dict[str, str]:
    if not message.status:
        return {}
    return {item.key: item.value for item in message.status[0].values}


def _number(values: dict[str, str], name: str, cast=float):
    try:
        return cast(values[name])
    except (KeyError, TypeError, ValueError):
        return None


class DistanceBenchmarkRecorder(Node):
    """Split synchronized observations by simulation-time target stage."""

    def __init__(self) -> None:
        super().__init__("target_perception_distance_benchmark")
        defaults = {
            "detections_topic": "/target_perception/detections_3d",
            "metrics_topic": "/target_perception/metrics",
            "output_dir": "/tmp/target_perception_distance_benchmark",
            "distances_m": [0.4, 0.6, 0.8, 1.0],
            "stage_duration_s": 10.0,
            "samples_per_distance": 100,
            "warmup_frames_per_distance": 15,
            "expected_frame": "base_link",
            "camera_x_m": 0.3306,
            "truth_y_m": 0.0,
            "truth_z_m": 0.4325,
            "truth_size_x_m": 0.20,
            "truth_size_y_m": 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.distances = tuple(float(value) for value in self.get_parameter("distances_m").value)
        self.stage_duration = float(self.get_parameter("stage_duration_s").value)
        self.samples_per_distance = int(self.get_parameter("samples_per_distance").value)
        self.warmup_frames = int(self.get_parameter("warmup_frames_per_distance").value)
        self.expected_frame = str(self.get_parameter("expected_frame").value)
        self.camera_x = float(self.get_parameter("camera_x_m").value)
        self.truth_y = float(self.get_parameter("truth_y_m").value)
        self.truth_z = float(self.get_parameter("truth_z_m").value)
        self.truth_size = (
            float(self.get_parameter("truth_size_x_m").value),
            float(self.get_parameter("truth_size_y_m").value),
        )
        if not self.distances or any(not 0.2 <= value <= 3.0 for value in self.distances):
            raise ValueError("distances_m must contain values in [0.2, 3.0]")
        if self.stage_duration <= 0.0 or self.samples_per_distance <= 0:
            raise ValueError("stage duration and sample count must be positive")

        self.stage_frames = [0 for _ in self.distances]
        self.rows: list[list[dict[str, object]]] = [[] for _ in self.distances]
        self.last_stage = -1
        self.finished = False
        detections_sub = Subscriber(
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
        self.sync = TimeSynchronizer([detections_sub, metrics_sub], queue_size=50)
        self.sync.registerCallback(self._callback)
        self.get_logger().info(
            "multi-distance benchmark ready: "
            f"distances={self.distances}, samples={self.samples_per_distance}, "
            f"stage={self.stage_duration:.1f}s"
        )

    def _callback(self, detections: Detection3DArray, diagnostics: DiagnosticArray) -> None:
        if self.finished:
            return
        sim_time = detections.header.stamp.sec + detections.header.stamp.nanosec * 1.0e-9
        stage = int(sim_time / self.stage_duration)
        if stage >= len(self.distances):
            return
        if stage != self.last_stage:
            if self.last_stage >= 0:
                self.get_logger().info(
                    f"stage {self.last_stage} collected "
                    f"{len(self.rows[self.last_stage])}/{self.samples_per_distance} samples"
                )
            self.last_stage = stage
        self.stage_frames[stage] += 1
        if self.stage_frames[stage] <= self.warmup_frames:
            return
        if len(self.rows[stage]) >= self.samples_per_distance:
            return

        diagnostic_values = _values(diagnostics)
        detected = bool(detections.detections and detections.detections[0].results)
        truth = (self.camera_x + self.distances[stage], self.truth_y, self.truth_z)
        row: dict[str, object] = {
            "sample_index": len(self.rows[stage]),
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
            "confidence": _number(diagnostic_values, "confidence"),
            "valid_depth_points": _number(diagnostic_values, "valid_depth_points", int),
            "valid_depth_ratio": _number(diagnostic_values, "valid_depth_ratio"),
        }
        if detected:
            detection = detections.detections[0]
            pose = detection.results[0].pose.pose.position
            position = (float(pose.x), float(pose.y), float(pose.z))
            error = tuple(value - target for value, target in zip(position, truth))
            size = (float(detection.bbox.size.x), float(detection.bbox.size.y))
            row.update(
                {
                    "x_m": position[0],
                    "y_m": position[1],
                    "z_m": position[2],
                    "error_x_m": error[0],
                    "error_y_m": error[1],
                    "error_z_m": error[2],
                    "position_error_m": math.sqrt(sum(value * value for value in error)),
                    "size_x_m": size[0],
                    "size_y_m": size[1],
                    "size_error_x_m": size[0] - self.truth_size[0],
                    "size_error_y_m": size[1] - self.truth_size[1],
                }
            )
            if detections.header.frame_id != self.expected_frame:
                self.get_logger().error(
                    f"unexpected frame {detections.header.frame_id!r}; "
                    f"expected {self.expected_frame!r}"
                )
        self.rows[stage].append(row)
        if all(len(items) >= self.samples_per_distance for items in self.rows):
            self._finish(request_shutdown=True)

    def _finish(self, request_shutdown: bool) -> None:
        self.finished = True
        self.output_dir.mkdir(parents=True, exist_ok=True)
        summaries = []
        for distance, rows in zip(self.distances, self.rows):
            label = f"{distance:.2f}".replace(".", "p")
            output_csv = self.output_dir / f"depth_{label}m.csv"
            _, _, artifact = write_artifacts(
                rows,
                output_csv,
                {
                    "expected_frame": self.expected_frame,
                    "truth_front_depth_m": distance,
                    "truth_position_m": [self.camera_x + distance, self.truth_y, self.truth_z],
                    "truth_size_m": list(self.truth_size),
                    "warmup_frames": self.warmup_frames,
                    "stage_duration_s": self.stage_duration,
                },
            )
            summaries.append(
                {"truth_front_depth_m": distance, **artifact["metrics"]}
            )

        summary_csv = self.output_dir / "distance_summary.csv"
        with summary_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
            writer.writeheader()
            writer.writerows(summaries)
        manifest = {
            "distances_m": list(self.distances),
            "samples_per_distance": self.samples_per_distance,
            "warmup_frames_per_distance": self.warmup_frames,
            "stage_duration_s": self.stage_duration,
            "distance_definition": "camera optical center to visible front surface",
        }
        with (self.output_dir / "manifest.json").open("w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        counts = [len(items) for items in self.rows]
        complete = all(count == self.samples_per_distance for count in counts)
        self.get_logger().info(
            f"DISTANCE BENCHMARK WRITTEN complete={complete} counts={counts} "
            f"summary={summary_csv}"
        )
        if request_shutdown:
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistanceBenchmarkRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.finished and any(node.rows):
            node._finish(request_shutdown=False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
