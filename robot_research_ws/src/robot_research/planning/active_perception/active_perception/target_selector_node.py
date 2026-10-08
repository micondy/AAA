"""ROS 2 adapter that locks one tracked object for active perception."""

from __future__ import annotations

from copy import deepcopy

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection3DArray

from .target_selector import TargetCandidate, TargetSelector


class TargetSelectorNode(Node):
    """Select one stable detection ID and publish its pose and lock state."""

    def __init__(self) -> None:
        super().__init__("target_selector")
        defaults = {
            "input_topic": "/target_perception/tracked_detections_3d",
            "pose_topic": "/active_perception/selected_target_pose",
            "id_topic": "/active_perception/selected_target_id",
            "visible_topic": "/active_perception/selected_target_visible",
            "detection_topic": "/active_perception/selected_target_detection",
            "target_id": "",
            "target_class": "*",
            "min_score": 0.5,
            "max_missed_frames": 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.selector = TargetSelector()
        self._last_state = ""
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            qos_profile_sensor_data,
        )
        self.id_pub = self.create_publisher(
            String, str(self.get_parameter("id_topic").value), 10
        )
        self.visible_pub = self.create_publisher(
            Bool, str(self.get_parameter("visible_topic").value), 10
        )
        self.detection_pub = self.create_publisher(
            Detection3DArray,
            str(self.get_parameter("detection_topic").value),
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            Detection3DArray,
            str(self.get_parameter("input_topic").value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _configure(self) -> None:
        self.selector.configure(
            str(self.get_parameter("target_id").value),
            str(self.get_parameter("target_class").value),
            float(self.get_parameter("min_score").value),
            int(self.get_parameter("max_missed_frames").value),
        )

    def _callback(self, message: Detection3DArray) -> None:
        try:
            self._configure()
            candidates = []
            for source_index, detection in enumerate(message.detections):
                if not detection.id or not detection.results:
                    continue
                result_index, result = max(
                    enumerate(detection.results),
                    key=lambda item: float(item[1].hypothesis.score),
                )
                position = result.pose.pose.position
                candidates.append(
                    TargetCandidate(
                        object_id=detection.id,
                        class_id=result.hypothesis.class_id,
                        score=float(result.hypothesis.score),
                        position=(position.x, position.y, position.z),
                        source_index=source_index,
                        result_index=result_index,
                    )
                )
            selection = self.selector.update(candidates)
        except ValueError as error:
            self.get_logger().error(f"invalid target selector input/config: {error}")
            return

        self.id_pub.publish(String(data=selection.selected_id or ""))
        self.visible_pub.publish(Bool(data=selection.visible))
        # On acquisition, publish the ID first and intentionally defer the pose
        # until the next frame. This lets TargetBeliefNode reset the old target
        # before it can receive a measurement from the new target.
        if selection.candidate is not None and not selection.selection_changed:
            source = message.detections[selection.candidate.source_index]
            result = source.results[selection.candidate.result_index]
            pose = PoseWithCovarianceStamped()
            pose.header = message.header
            pose.pose = result.pose
            self.pose_pub.publish(pose)
            selected_detection = Detection3DArray()
            selected_detection.header = message.header
            selected_detection.detections.append(deepcopy(source))
            self.detection_pub.publish(selected_detection)

        if selection.selection_changed or selection.state != self._last_state:
            self.get_logger().info(
                f"target_selector state={selection.state}, "
                f"selected_id={selection.selected_id or '<none>'}, "
                f"missed_frames={selection.missed_frames}"
            )
        self._last_state = selection.state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetSelectorNode()
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
