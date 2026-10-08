"""Explicitly armed Nav2 executor for a previously planned best view."""

from __future__ import annotations

from datetime import datetime
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger

from .nbv_execution import make_belief_snapshot, summarize_belief_change


def target_replacement_detected(execution_target_id: str, new_target_id: str) -> bool:
    """Return true only for an explicit different target, not temporary loss."""
    active = execution_target_id.strip()
    update = new_target_id.strip()
    return bool(active and update and update != active)


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class NBVExecutorNode(Node):
    """Send the latest best view only after an explicit, gated service call."""

    def __init__(self) -> None:
        super().__init__("nbv_executor")
        defaults = {
            "execution_enabled": False,
            "best_pose_topic": "/active_perception/best_view_pose",
            "belief_topic": "/active_perception/target_belief",
            "observation_count_topic": "/active_perception/target_observation_count",
            "visible_topic": "/active_perception/target_visible",
            "target_id_topic": "/active_perception/selected_target_id",
            "navigate_action": "/navigate_to_pose",
            "goal_max_age_seconds": 120.0,
            "navigation_timeout_seconds": 120.0,
            "post_observation_timeout_seconds": 10.0,
            "minimum_new_observations": 10,
            "output_directory": "/tmp/active_perception_executions",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.execution_enabled = bool(self.get_parameter("execution_enabled").value)
        self.best_pose: PoseStamped | None = None
        self.best_pose_receipt = 0.0
        self.best_pose_revision = 0
        self.last_executed_revision = 0
        self.belief: PoseWithCovarianceStamped | None = None
        self.observation_count = 0
        self.visible = False
        self.target_id = ""
        self.execution_target_id = ""
        self.before_snapshot = None
        self.goal_handle = None
        self.started_monotonic = 0.0
        self.navigation_finished_monotonic = 0.0
        self.navigation_status = "IDLE"
        self.navigation_error_code = 0
        self.state = "DISARMED" if not self.execution_enabled else "READY"

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("best_pose_topic").value),
            self._best_pose_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("belief_topic").value),
            self._belief_callback,
            10,
        )
        self.create_subscription(
            UInt32,
            str(self.get_parameter("observation_count_topic").value),
            self._count_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("visible_topic").value),
            self._visible_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_id_topic").value),
            self._target_id_callback,
            10,
        )
        self.state_pub = self.create_publisher(
            String, "/active_perception/execution_state", 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/active_perception/execution_diagnostics", 10
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
        )
        self.execute_service = self.create_service(
            Trigger, "/active_perception/execute_best_view", self._execute
        )
        self.cancel_service = self.create_service(
            Trigger, "/active_perception/cancel_view_execution", self._cancel
        )
        self.timer = self.create_timer(0.1, self._monitor)
        self._publish_state()

    def _best_pose_callback(self, message: PoseStamped) -> None:
        self.best_pose = message
        self.best_pose_receipt = time.monotonic()
        self.best_pose_revision += 1

    def _belief_callback(self, message: PoseWithCovarianceStamped) -> None:
        self.belief = message

    def _count_callback(self, message: UInt32) -> None:
        self.observation_count = int(message.data)

    def _visible_callback(self, message: Bool) -> None:
        self.visible = bool(message.data)

    def _target_id_callback(self, message: String) -> None:
        new_target_id = message.data.strip()
        if self.state in ("NAVIGATING", "OBSERVING") and target_replacement_detected(
            self.execution_target_id, new_target_id
        ):
            self.get_logger().error("target changed during NBV execution; canceling")
            self._request_cancel("TARGET_CHANGED")
        self.target_id = new_target_id

    def _set_response(self, response, success: bool, message: str):
        response.success = success
        response.message = message
        return response

    def _execute(self, _request: Trigger.Request, response: Trigger.Response):
        if not self.execution_enabled:
            return self._set_response(
                response,
                False,
                "execution disabled; restart with enable_nbv_execution:=true",
            )
        if self.state in ("NAVIGATING", "OBSERVING", "CANCELING"):
            return self._set_response(response, False, "execution already in progress")
        if self.best_pose is None:
            return self._set_response(response, False, "no best view has been planned")
        if self.best_pose_revision == self.last_executed_revision:
            return self._set_response(response, False, "latest best view was already executed")
        age = time.monotonic() - self.best_pose_receipt
        max_age = float(self.get_parameter("goal_max_age_seconds").value)
        if age > max_age:
            return self._set_response(
                response, False, f"best view is stale ({age:.1f}s > {max_age:.1f}s)"
            )
        if self.belief is None or not self.target_id:
            return self._set_response(response, False, "target belief or target ID missing")
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            return self._set_response(response, False, "Nav2 NavigateToPose unavailable")

        try:
            self.before_snapshot = make_belief_snapshot(
                self.belief, self.observation_count, self.visible
            )
        except ValueError as error:
            return self._set_response(response, False, str(error))

        goal = NavigateToPose.Goal()
        goal.pose = self.best_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.execution_target_id = self.target_id
        self.started_monotonic = time.monotonic()
        self.last_executed_revision = self.best_pose_revision
        self.state = "NAVIGATING"
        self.navigation_status = "SENDING"
        self._publish_state()
        future = self.navigate_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response)
        return self._set_response(
            response,
            True,
            f"sent revision {self.best_pose_revision} for target {self.target_id}",
        )

    def _feedback_callback(self, message) -> None:
        feedback = message.feedback
        self.last_distance_remaining = float(feedback.distance_remaining)
        self.last_recoveries = int(feedback.number_of_recoveries)

    def _goal_response(self, future) -> None:
        try:
            self.goal_handle = future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                self._finish("REJECTED", "Nav2 rejected the best-view goal")
                return
            self.navigation_status = "EXECUTING"
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self._navigation_result)
        except Exception as error:
            self._finish("SEND_FAILED", str(error))

    def _navigation_result(self, future) -> None:
        try:
            wrapped = future.result()
            self.navigation_status = STATUS_NAMES.get(wrapped.status, str(wrapped.status))
            self.navigation_error_code = int(wrapped.result.error_code)
        except Exception as error:
            self._finish("RESULT_FAILED", str(error))
            return
        if self.navigation_status != "SUCCEEDED":
            status = (
                getattr(self, "cancel_reason", self.navigation_status)
                if self.navigation_status == "CANCELED"
                else self.navigation_status
            )
            self._finish(status, "navigation did not succeed")
            return
        self.navigation_finished_monotonic = time.monotonic()
        self.state = "OBSERVING"
        self._publish_state()

    def _monitor(self) -> None:
        now = time.monotonic()
        if self.state == "NAVIGATING":
            timeout = float(self.get_parameter("navigation_timeout_seconds").value)
            if now - self.started_monotonic > timeout:
                self.get_logger().error("NBV navigation timed out; canceling")
                self._request_cancel("NAVIGATION_TIMEOUT")
        elif self.state == "OBSERVING":
            assert self.before_snapshot is not None
            new_count = self.observation_count - self.before_snapshot.observation_count
            required = int(self.get_parameter("minimum_new_observations").value)
            if new_count >= required and self.belief is not None:
                self._finish("SUCCEEDED", "post-view observations collected", record=True)
                return
            timeout = float(
                self.get_parameter("post_observation_timeout_seconds").value
            )
            if now - self.navigation_finished_monotonic > timeout:
                self._finish(
                    "OBSERVATION_TIMEOUT",
                    f"received {new_count}/{required} post-view observations",
                    record=True,
                )

    def _cancel(self, _request: Trigger.Request, response: Trigger.Response):
        if self.state not in ("NAVIGATING", "OBSERVING"):
            return self._set_response(response, False, "no active execution")
        self._request_cancel("USER_CANCELED")
        return self._set_response(response, True, "cancel requested")

    def _request_cancel(self, reason: str) -> None:
        self.cancel_reason = reason
        if self.goal_handle is not None and self.state == "NAVIGATING":
            self.state = "CANCELING"
            self._publish_state()
            self.goal_handle.cancel_goal_async()
        else:
            self._finish(reason, "execution stopped")

    def _finish(self, status: str, message: str, record: bool = False) -> None:
        summary = None
        if record and self.before_snapshot is not None and self.belief is not None:
            after = make_belief_snapshot(
                self.belief, self.observation_count, self.visible
            )
            summary = summarize_belief_change(self.before_snapshot, after)
        artifact = self._write_artifact(status, message, summary)
        self.state = "COMPLETE" if status == "SUCCEEDED" else "FAILED"
        self._publish_state(status=status, message=message, summary=summary, artifact=artifact)
        self.get_logger().info(
            f"NBV execution {status}: {message}; artifact={artifact or '<none>'}"
        )
        self.goal_handle = None

    def _write_artifact(self, status: str, message: str, summary) -> str:
        if self.best_pose is None:
            return ""
        pose = self.best_pose.pose
        record = {
            "schema_version": 1,
            "created_at": datetime.now().astimezone().isoformat(),
            "status": status,
            "message": message,
            "target_id": self.execution_target_id,
            "goal": {
                "frame_id": self.best_pose.header.frame_id,
                "position": {"x": pose.position.x, "y": pose.position.y},
                "orientation": {"z": pose.orientation.z, "w": pose.orientation.w},
            },
            "navigation_duration_seconds": max(
                0.0, time.monotonic() - self.started_monotonic
            ),
            "navigation_status": self.navigation_status,
            "navigation_error_code": self.navigation_error_code,
            "distance_remaining_m": getattr(self, "last_distance_remaining", math.nan),
            "recoveries": getattr(self, "last_recoveries", 0),
            "belief_change": summary,
        }
        output = Path(str(self.get_parameter("output_directory").value))
        output.mkdir(parents=True, exist_ok=True)
        filename = datetime.now().strftime("nbv_execution_%Y%m%d_%H%M%S.json")
        path = output / filename
        path.write_text(json.dumps(record, indent=2, allow_nan=True), encoding="utf-8")
        return str(path)

    def _publish_state(
        self, status: str = "", message: str = "", summary=None, artifact: str = ""
    ) -> None:
        self.state_pub.publish(String(data=self.state))
        diagnostic = DiagnosticStatus()
        diagnostic.name = "active_perception/nbv_executor"
        diagnostic.hardware_id = "software"
        diagnostic.level = (
            DiagnosticStatus.ERROR if self.state == "FAILED" else DiagnosticStatus.OK
        )
        diagnostic.message = status or self.state.lower()
        values = {
            "execution_enabled": self.execution_enabled,
            "state": self.state,
            "target_id": self.execution_target_id or self.target_id,
            "detail": message,
            "artifact": artifact,
        }
        if summary is not None:
            values["new_observations"] = summary["new_observations"]
            values["covariance_trace_reduction"] = summary[
                "covariance_trace_reduction"
            ]
            values["covariance_trace_reduction_ratio"] = summary[
                "covariance_trace_reduction_ratio"
            ]
        diagnostic.values = [
            KeyValue(key=key, value=str(value)) for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(diagnostic)
        self.diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NBVExecutorNode()
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
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
