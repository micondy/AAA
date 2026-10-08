"""Record a fixed number of Target Belief outputs and convergence metrics."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .belief_benchmark import write_artifacts
from .target_belief_node import position_covariance


class BeliefBenchmarkNode(Node):
    """Collect a bounded, reproducible belief convergence artifact."""

    def __init__(self) -> None:
        super().__init__("belief_benchmark")
        defaults = {
            "belief_topic": "/active_perception/target_belief",
            "output_csv": "/tmp/target_belief.csv",
            "sample_count": 100,
            "warmup_frames": 15,
            "truth_x_m": 1.1006,
            "truth_y_m": 0.0,
            "truth_z_m": 0.4325,
            "expected_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.output_csv = Path(str(self.get_parameter("output_csv").value))
        self.sample_count = int(self.get_parameter("sample_count").value)
        self.warmup_remaining = int(self.get_parameter("warmup_frames").value)
        self.truth = (
            float(self.get_parameter("truth_x_m").value),
            float(self.get_parameter("truth_y_m").value),
            float(self.get_parameter("truth_z_m").value),
        )
        self.expected_frame = str(self.get_parameter("expected_frame").value)
        if self.sample_count <= 0 or self.warmup_remaining < 0:
            raise ValueError("sample_count must be positive and warmup_frames non-negative")
        self.rows: list[dict[str, float]] = []
        self.completed = False
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("belief_topic").value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message: PoseWithCovarianceStamped) -> None:
        if self.completed:
            return
        if message.header.frame_id != self.expected_frame:
            self.get_logger().error(
                f"expected frame {self.expected_frame}, got {message.header.frame_id}"
            )
            return
        if self.warmup_remaining > 0:
            self.warmup_remaining -= 1
            return
        position = message.pose.pose.position
        error = math.sqrt(
            (position.x - self.truth[0]) ** 2
            + (position.y - self.truth[1]) ** 2
            + (position.z - self.truth[2]) ** 2
        )
        covariance = position_covariance(message)
        self.rows.append(
            {
                "sample": len(self.rows) + 1,
                "stamp_seconds": (
                    message.header.stamp.sec
                    + message.header.stamp.nanosec * 1.0e-9
                ),
                "x_m": position.x,
                "y_m": position.y,
                "z_m": position.z,
                "position_error_m": error,
                "variance_x_m2": covariance[0, 0],
                "variance_y_m2": covariance[1, 1],
                "variance_z_m2": covariance[2, 2],
                "covariance_trace_m2": float(covariance.trace()),
            }
        )
        if len(self.rows) >= self.sample_count:
            artifact = write_artifacts(
                self.output_csv,
                self.rows,
                {
                    "belief_frame": self.expected_frame,
                    "truth_position_m": list(self.truth),
                    "warmup_frames": int(
                        self.get_parameter("warmup_frames").value
                    ),
                },
            )
            metrics = artifact["metrics"]
            self.get_logger().info(
                f"belief benchmark complete: samples={self.sample_count}, "
                f"rmse={metrics['position_error_rmse_m']:.6f} m, "
                f"final_trace={metrics['final_covariance_trace_m2']:.8f} m^2, "
                f"output={self.output_csv}"
            )
            self.completed = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BeliefBenchmarkNode()
    try:
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
