"""ROS 2 node that maintains a target-position belief in one fixed frame."""

from __future__ import annotations

from collections import deque

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose_with_covariance_stamped
from tf2_ros import Buffer, TransformException, TransformListener

from .belief import TargetBeliefFilter


def position_covariance(message: PoseWithCovarianceStamped) -> np.ndarray:
    """Extract the XYZ covariance block from a ROS 6x6 pose covariance."""
    covariance = message.pose.covariance
    return np.array(
        [
            [covariance[0], covariance[1], covariance[2]],
            [covariance[6], covariance[7], covariance[8]],
            [covariance[12], covariance[13], covariance[14]],
        ],
        dtype=float,
    )


def assign_position_covariance(
    message: PoseWithCovarianceStamped,
    covariance: np.ndarray,
    orientation_variance: float,
) -> None:
    """Write XYZ covariance and explicitly mark orientation as uninformative."""
    output = [0.0] * 36
    for row in range(3):
        for column in range(3):
            output[row * 6 + column] = float(covariance[row, column])
    output[21] = orientation_variance
    output[28] = orientation_variance
    output[35] = orientation_variance
    message.pose.covariance = output


class TargetBeliefNode(Node):
    """Transform, gate and fuse target observations."""

    def __init__(self) -> None:
        super().__init__("target_belief")
        defaults = {
            "input_topic": "/target_perception/target_pose",
            "output_topic": "/active_perception/target_belief",
            "count_topic": "/active_perception/target_observation_count",
            "rejected_count_topic": "/active_perception/target_rejected_count",
            "visible_topic": "/active_perception/target_visible",
            "selection_id_topic": "/active_perception/selected_target_id",
            "belief_frame": "base_link",
            "variance_floor": 1.0e-6,
            "variance_ceiling": 0.25,
            "process_variance_per_second": 1.0e-6,
            "gate_mahalanobis_squared": 11.344867,
            "visibility_timeout_seconds": 0.5,
            "transform_timeout_seconds": 0.1,
            "transform_queue_timeout_seconds": 2.0,
            "transform_queue_max_messages": 50,
            "orientation_variance": 1.0e6,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.belief_frame = str(self.get_parameter("belief_frame").value)
        self.visibility_timeout = float(
            self.get_parameter("visibility_timeout_seconds").value
        )
        self.transform_timeout = float(
            self.get_parameter("transform_timeout_seconds").value
        )
        self.transform_queue_timeout = float(
            self.get_parameter("transform_queue_timeout_seconds").value
        )
        self.transform_queue_max_messages = int(
            self.get_parameter("transform_queue_max_messages").value
        )
        if self.transform_queue_timeout <= 0.0:
            raise ValueError("transform_queue_timeout_seconds must be positive")
        if self.transform_queue_max_messages <= 0:
            raise ValueError("transform_queue_max_messages must be positive")
        self.orientation_variance = float(
            self.get_parameter("orientation_variance").value
        )
        self.filter = TargetBeliefFilter(
            variance_floor=float(self.get_parameter("variance_floor").value),
            variance_ceiling=float(self.get_parameter("variance_ceiling").value),
            process_variance_per_second=float(
                self.get_parameter("process_variance_per_second").value
            ),
            gate_mahalanobis_squared=float(
                self.get_parameter("gate_mahalanobis_squared").value
            ),
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.count_pub = self.create_publisher(
            UInt32, str(self.get_parameter("count_topic").value), 10
        )
        self.rejected_count_pub = self.create_publisher(
            UInt32, str(self.get_parameter("rejected_count_topic").value), 10
        )
        self.visible_pub = self.create_publisher(
            Bool, str(self.get_parameter("visible_topic").value), 10
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("input_topic").value),
            self._measurement_callback,
            qos_profile_sensor_data,
        )
        self.selection_subscription = self.create_subscription(
            String,
            str(self.get_parameter("selection_id_topic").value),
            self._selection_callback,
            10,
        )
        self.reset_service = self.create_service(
            Trigger, "/active_perception/reset_target_belief", self._reset
        )
        self.last_accepted_receipt: Time | None = None
        self.current_target_id: str | None = None
        self.pending_measurements = deque()
        self.visibility_timer = self.create_timer(0.1, self._publish_visibility)
        self.transform_retry_timer = self.create_timer(
            0.02, self._retry_pending_measurements
        )

    def _clear_belief(self) -> None:
        self.filter.reset()
        self.pending_measurements.clear()
        self.last_accepted_receipt = None
        self.count_pub.publish(UInt32(data=0))
        self.rejected_count_pub.publish(UInt32(data=0))
        self.visible_pub.publish(Bool(data=False))

    def _selection_callback(self, message: String) -> None:
        selected_id = message.data.strip() or None
        if selected_id == self.current_target_id:
            return
        previous_id = self.current_target_id
        self.current_target_id = selected_id
        self._clear_belief()
        self.get_logger().info(
            f"target selection changed: {previous_id or '<none>'} -> "
            f"{selected_id or '<none>'}; belief reset"
        )

    def _in_belief_frame(
        self, message: PoseWithCovarianceStamped
    ) -> PoseWithCovarianceStamped | None:
        if message.header.frame_id == self.belief_frame:
            return message
        try:
            transform = self.tf_buffer.lookup_transform(
                self.belief_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
        except TransformException as error:
            self.last_transform_error = str(error)
            return None
        return do_transform_pose_with_covariance_stamped(message, transform)

    def _measurement_callback(self, message: PoseWithCovarianceStamped) -> None:
        if self.pending_measurements:
            self._queue_measurement(message)
            return
        transformed = self._in_belief_frame(message)
        if transformed is None:
            self._queue_measurement(message)
            return
        self._update_filter(transformed)

    def _queue_measurement(self, message: PoseWithCovarianceStamped) -> None:
        if len(self.pending_measurements) >= self.transform_queue_max_messages:
            self.pending_measurements.popleft()
            self.get_logger().warning(
                "TF measurement queue full; dropped oldest target observation"
            )
        self.pending_measurements.append((message, self.get_clock().now()))

    def _retry_pending_measurements(self) -> None:
        processed = 0
        while self.pending_measurements and processed < 20:
            message, receipt_time = self.pending_measurements[0]
            transformed = self._in_belief_frame(message)
            if transformed is None:
                age = self.get_clock().now() - receipt_time
                if age.nanoseconds * 1.0e-9 <= self.transform_queue_timeout:
                    return
                self.pending_measurements.popleft()
                self.get_logger().warning(
                    "dropping target observation after TF queue timeout: "
                    f"{message.header.frame_id}->{self.belief_frame}: "
                    f"{getattr(self, 'last_transform_error', 'transform unavailable')}"
                )
                continue
            self.pending_measurements.popleft()
            self._update_filter(transformed)
            processed += 1

    def _update_filter(self, transformed: PoseWithCovarianceStamped) -> None:
        position = transformed.pose.pose.position
        stamp_seconds = (
            transformed.header.stamp.sec
            + transformed.header.stamp.nanosec * 1.0e-9
        )
        try:
            result = self.filter.update(
                np.array((position.x, position.y, position.z)),
                position_covariance(transformed),
                stamp_seconds,
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().warning(f"dropping invalid observation: {error}")
            return

        if not result.accepted:
            if result.snapshot is not None:
                self.rejected_count_pub.publish(
                    UInt32(data=result.snapshot.rejected_count)
                )
            self.get_logger().warning(
                f"rejected observation: reason={result.reason}, "
                f"mahalanobis_squared={result.mahalanobis_squared:.3f}"
            )
            return
        snapshot = result.snapshot
        assert snapshot is not None
        output = PoseWithCovarianceStamped()
        output.header.stamp = transformed.header.stamp
        output.header.frame_id = self.belief_frame
        output.pose.pose.position.x = float(snapshot.mean[0])
        output.pose.pose.position.y = float(snapshot.mean[1])
        output.pose.pose.position.z = float(snapshot.mean[2])
        output.pose.pose.orientation.w = 1.0
        assign_position_covariance(
            output, snapshot.covariance, self.orientation_variance
        )
        self.pose_pub.publish(output)
        self.count_pub.publish(UInt32(data=snapshot.accepted_count))
        self.rejected_count_pub.publish(UInt32(data=snapshot.rejected_count))
        self.last_accepted_receipt = self.get_clock().now()
        self.visible_pub.publish(Bool(data=True))
        if snapshot.accepted_count == 1 or snapshot.accepted_count % 100 == 0:
            self.get_logger().info(
                f"belief accepted_count={snapshot.accepted_count}, "
                f"mean=({snapshot.mean[0]:.4f}, {snapshot.mean[1]:.4f}, "
                f"{snapshot.mean[2]:.4f}), "
                f"covariance_trace={np.trace(snapshot.covariance):.8f}"
            )

    def _publish_visibility(self) -> None:
        if self.last_accepted_receipt is None:
            self.visible_pub.publish(Bool(data=False))
            return
        elapsed = self.get_clock().now() - self.last_accepted_receipt
        self.visible_pub.publish(
            Bool(data=elapsed.nanoseconds * 1.0e-9 <= self.visibility_timeout)
        )

    def _reset(self, _request: Trigger.Request, response: Trigger.Response):
        self._clear_belief()
        response.success = True
        response.message = "target belief reset"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetBeliefNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
