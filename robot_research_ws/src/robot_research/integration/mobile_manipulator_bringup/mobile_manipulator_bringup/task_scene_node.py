"""Apply or remove a configured box environment through MoveIt's service."""

from __future__ import annotations

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from mobile_manipulator_bringup.task_scene import load_task_scene


def _collision_object(frame_id, box, operation: int) -> CollisionObject:
    message = CollisionObject()
    message.header.frame_id = frame_id
    message.id = box.object_id
    message.operation = operation
    if operation == CollisionObject.ADD:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(box.size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = box.pose[:3]
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = box.pose[3:]
        message.primitives.append(primitive)
        message.primitive_poses.append(pose)
    return message


class TaskSceneApplicator(Node):
    """Send one deterministic environment diff to MoveIt."""

    def __init__(self) -> None:
        super().__init__("task_scene_applicator")
        default_scene = (
            Path(get_package_share_directory("mobile_manipulator_bringup"))
            / "config"
            / "generic_task_scene.yaml"
        )
        self.declare_parameter("scene_file", str(default_scene))
        self.declare_parameter("operation", "add")
        self.declare_parameter("service_timeout_s", 15.0)
        self.scene = load_task_scene(str(self.get_parameter("scene_file").value))
        operation_name = str(self.get_parameter("operation").value).strip().lower()
        if operation_name not in ("add", "remove"):
            raise ValueError("operation must be 'add' or 'remove'")
        self.operation = (
            CollisionObject.ADD if operation_name == "add" else CollisionObject.REMOVE
        )
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def run(self) -> bool:
        timeout = float(self.get_parameter("service_timeout_s").value)
        if not self.client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/apply_planning_scene is unavailable")
            return False
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        planning_scene.world.collision_objects = [
            _collision_object(self.scene.frame_id, box, self.operation)
            for box in self.scene.objects
        ]
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error("MoveIt rejected the task-scene diff")
            return False
        action = "added" if self.operation == CollisionObject.ADD else "removed"
        self.get_logger().info(
            f"task scene {action}: frame={self.scene.frame_id}, "
            f"objects={[box.object_id for box in self.scene.objects]}"
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskSceneApplicator()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)
