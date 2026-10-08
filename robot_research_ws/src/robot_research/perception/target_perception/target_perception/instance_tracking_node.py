"""ROS 2 adapter for stable class-aware 3D instance IDs."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection3DArray

from target_perception.instance_tracking import (
    ClassAware3DTracker,
    InstanceObservation,
)


class InstanceTrackingNode(Node):
    """Rewrite frame-local detection IDs with baseline cross-frame IDs."""

    def __init__(self) -> None:
        super().__init__("instance_tracking")
        self.declare_parameter("input_topic", "/target_perception/detections_3d")
        self.declare_parameter(
            "output_topic", "/target_perception/tracked_detections_3d"
        )
        self.declare_parameter("max_distance_m", 0.35)
        self.declare_parameter("max_missed_frames", 5)
        self.tracker = ClassAware3DTracker(
            max_distance_m=float(self.get_parameter("max_distance_m").value),
            max_missed_frames=int(
                self.get_parameter("max_missed_frames").value
            ),
        )
        self.publisher = self.create_publisher(
            Detection3DArray,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            Detection3DArray,
            str(self.get_parameter("input_topic").value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message: Detection3DArray) -> None:
        valid = [detection for detection in message.detections if detection.results]
        observations = []
        for detection in valid:
            result = detection.results[0]
            position = result.pose.pose.position
            observations.append(
                InstanceObservation(
                    class_id=result.hypothesis.class_id,
                    position=(float(position.x), float(position.y), float(position.z)),
                )
            )
        stable_ids = self.tracker.update(observations)
        output = Detection3DArray()
        output.header = message.header
        for detection, stable_id in zip(valid, stable_ids):
            tracked = deepcopy(detection)
            tracked.id = stable_id
            output.detections.append(tracked)
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InstanceTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
