# SPDX-License-Identifier: Apache-2.0
"""FollowJointTrajectory action server for the Nero MuJoCo bridge.

MoveIt drives the arm by sending ``control_msgs/FollowJointTrajectory`` goals
to ``/<controller_name>/follow_joint_trajectory`` (see the MoveIt config's
``moveit_controllers.yaml``: ``arm_controller`` and ``gripper_controller``).
This module implements that action on top of the runtime's position actuators
so the same MoveIt graph can drive either this simulation or a real ros2_control
controller.

Each server owns one action name and a fixed ``joint_names`` order, plus two
callbacks supplied by the caller:

- ``apply(positions)``: write one trajectory point's targets (called under the
  bridge lock), in ``joint_names`` order.
- ``read()``: return the measured positions in the same order (for feedback).

The trajectory is stepped in real time: each point is applied once its
``time_from_start`` elapses relative to goal acceptance. The physics loop here
publishes wall-clock ``/clock`` and advances in real time, so
``time_from_start`` maps directly to elapsed wall time.

Settling: even with the retuned servo gains (see runtime.py / the main
README's "Servo retune" section — the vendored ``damping=2000`` gains are far
more overdamped than this), the arm still takes a couple of seconds to reach
a commanded pose. This executor therefore reports success once the final
point has been *commanded*, rather than blocking until the pose is physically
reached. The true pose stays visible on ``/joint_states``, so MoveIt plans each
subsequent goal from the real state.
"""

from __future__ import annotations

import logging
import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse

logger = logging.getLogger(__name__)

# Poll interval for cancellation/shutdown while pacing between trajectory
# points. Small enough to keep goal timestamps accurate, large enough to avoid
# busy-waiting.
_SLEEP_GRANULARITY_S = 0.005


def _duration_to_seconds(duration) -> float:
    """builtin_interfaces/Duration -> float seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class FollowJointTrajectoryServer:
    """One FollowJointTrajectory action server bound to a fixed joint set.

    ``apply`` is called under ``lock`` with one point's positions (in
    ``joint_names`` order); ``read`` returns the measured positions in the same
    order. Both are plain callables so the bridge can adapt the arm (raw joint
    targets) and gripper (two mirrored prismatic joints -> one width) without
    this class knowing about the runtime.
    """

    def __init__(self, node, action_name, joint_names, apply, read, *,
                 lock, stop_event):
        self._node = node
        self._action_name = action_name
        self.joint_names = list(joint_names)
        self._apply = apply
        self._read = read
        self._lock = lock
        self._stop_event = stop_event

        self._action_server = ActionServer(
            node,
            FollowJointTrajectory,
            action_name,
            self._on_execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )

    def _on_goal(self, goal_request):
        trajectory = goal_request.trajectory
        if list(trajectory.joint_names) != self.joint_names:
            logger.error(
                "%s: rejecting goal, joint_names %s != expected %s",
                self._action_name, list(trajectory.joint_names), self.joint_names,
            )
            return GoalResponse.REJECT
        if not trajectory.points:
            logger.error("%s: rejecting goal with no trajectory points", self._action_name)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, cancel_request):
        logger.info("%s: cancel accepted", self._action_name)
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle):
        points = goal_handle.request.trajectory.points
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(goal_handle.request.trajectory.joint_names)

        start = time.monotonic()
        for point in points:
            if not self._wait_until(start, point.time_from_start, goal_handle):
                # Canceled or shutting down; the handle has already been
                # transitioned, the returned result is ignored.
                return FollowJointTrajectory.Result()

            positions = [float(p) for p in point.positions]
            with self._lock:
                self._apply(positions)
            with self._lock:
                measured = self._read()

            feedback.header.stamp = self._node.get_clock().now().to_msg()
            feedback.desired.positions = positions
            feedback.actual.positions = measured
            feedback.error.positions = [d - a for d, a in zip(positions, measured)]
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        return FollowJointTrajectory.Result(
            error_code=FollowJointTrajectory.Result.SUCCESSFUL
        )

    def _wait_until(self, start, time_from_start, goal_handle):
        """Sleep until ``time_from_start`` seconds after ``start``.

        Returns False (and transitions the goal) if the goal is canceled or the
        node is shutting down; True once the deadline is reached.
        """
        deadline = start + _duration_to_seconds(time_from_start)
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            if self._stop_event.is_set():
                goal_handle.abort()
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(remaining, _SLEEP_GRANULARITY_S))
