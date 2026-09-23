"""ROS 2 bridge between the Nero Native MuJoCo runtime and the Robonix graph.

This is the ROS boundary: it owns the MuJoCo physics loop (via
`native.runtime.NeroRuntime`) and publishes the authoritative state topics,
while subscribing to command topics and exposing control services. The
Robonix arm primitive (primitives/nero_arm) does NOT publish its own copies
of this state — it only declares these topics on Atlas — so there is exactly
one publisher of each feedback topic.

Published (topic_out):
    /joint_states        sensor_msgs/JointState   joint1..joint7 + gripper
    /feedback/tcp_pose   geometry_msgs/Pose       end-effector pose (fingers midpoint)
    /feedback/object_pose geometry_msgs/PoseStamped grasp-target pose (world; only when
                                                  the loaded model has an `object` body)
    /clock               rosgraph_msgs/Clock      wall-clock for use_sim_time consumers
    /tf                  tf2_msgs/TFMessage       world->base_link (static) + arm links

Subscribed (topic_in):
    /joint_command       sensor_msgs/JointState   named joint target (arm + gripper)
    /control/move_j      sensor_msgs/JointState   agx_arm_ctrl-compatible alias

Services:
    /move_home           std_srvs/Empty
    /emergency_stop      std_srvs/Empty
    /control_enable      std_srvs/SetBool

Actions (MoveIt interface, see moveit_controllers.yaml):
    /arm_controller/follow_joint_trajectory      joint1..joint7
    /gripper_controller/follow_joint_trajectory  gripper_joint1/gripper_joint2 -> finger width

Run via sim/start.sh, or directly:
    source /opt/ros/humble/setup.bash
    python sim/bridge/bridge_node.py
"""

from __future__ import annotations

import argparse
import os
import sys
import threading

import mujoco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool
from tf2_msgs.msg import TFMessage

# Make `native` importable from this file's sibling package.
_SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

from native.controller import ARM_JOINT_NAMES, GRIPPER_JOINT_NAME  # noqa: E402
from native.runtime import NeroRuntime, ARM_BODY_NAMES, FINGER_BODY_NAMES  # noqa: E402
from native.scene_builder import build_scene  # noqa: E402
from bridge.follow_joint_trajectory import FollowJointTrajectoryServer  # noqa: E402

# MoveIt gripper controller joints (mirrored by an <equality> constraint in
# the MJCF, driven by the single `gripper` actuator). MoveIt plans these two
# prismatic joints; the bridge folds them back into one finger width.
GRIPPER_JOINT_NAMES = ("gripper_joint1", "gripper_joint2")

# Body name of the grasp target in the tabletop scene. Absent in the bare-arm
# model; the object-pose publisher is skipped when it is not present.
OBJECT_BODY_NAME = "object"


def _ros_quat(mj_quat):
    """MuJoCo [w,x,y,z] -> ROS geometry_msgs/Quaternion [x,y,z,w]."""
    return Quaternion(x=float(mj_quat[1]), y=float(mj_quat[2]),
                      z=float(mj_quat[3]), w=float(mj_quat[0]))


class NeroBridgeNode(Node):
    """Bridges NeroRuntime physics to ROS 2 topics/services/tf."""

    def __init__(self, model_path, *, sim_hz=500.0, pub_rate_hz=100.0,
                 use_viewer=False):
        super().__init__("nero_mujoco_bridge")

        self.runtime = NeroRuntime(model_path, sim_hz=sim_hz)
        self._model = self.runtime.model
        self._data = self.runtime.data

        self._lock = threading.Lock()
        self.control_enabled = True

        if use_viewer:
            # `import mujoco` does not pull in the viewer submodule (it needs
            # glfw); import it explicitly before calling launch_passive. Bind it
            # under a distinct name so the local binding does not shadow the
            # module-level `mujoco` used for the body-id lookups below.
            import mujoco.viewer as mujoco_viewer  # noqa: F401
            self._viewer = mujoco_viewer.launch_passive(self._model, self._data)
        else:
            self._viewer = None

        # Topic_out publishers.
        self._joint_states_pub = self.create_publisher(JointState, "/joint_states", 1)
        self._tcp_pose_pub = self.create_publisher(Pose, "/feedback/tcp_pose", 1)
        self._object_pose_pub = self.create_publisher(PoseStamped, "/feedback/object_pose", 1)
        self._clock_pub = self.create_publisher(Clock, "/clock", 1)
        self._tf_pub = self.create_publisher(TFMessage, "/tf", 1)

        # Topic_in subscriptions.
        self.create_subscription(JointState, "/joint_command", self._joint_command_callback, 1)
        self.create_subscription(JointState, "/control/move_j", self._joint_command_callback, 1)

        # Control services.
        self.create_service(Empty, "/move_home", self._move_home_callback)
        self.create_service(Empty, "/emergency_stop", self._emergency_stop_callback)
        self.create_service(SetBool, "/control_enable", self._control_enable_callback)

        # Static TF: world -> base_link (fixed base, identity).
        self._static_tf = self._make_static_tf()

        # Grasp-target body id, or -1 when the loaded model has no object.
        self._object_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY_NAME
        )

        self._sim_period = 1.0 / sim_hz
        self._pub_period = 1.0 / pub_rate_hz
        self._stop_event = threading.Event()

        # MoveIt FollowJointTrajectory actions (see moveit_controllers.yaml).
        # `arm_controller` commands joint1..joint7; `gripper_controller` is
        # folded from the two mirrored gripper joints into a single width.
        self._arm_action = FollowJointTrajectoryServer(
            self,
            "/arm_controller/follow_joint_trajectory",
            list(ARM_JOINT_NAMES),
            apply=self.runtime.move_j,
            read=self._read_arm_positions,
            lock=self._lock,
            stop_event=self._stop_event,
        )
        self._gripper_action = FollowJointTrajectoryServer(
            self,
            "/gripper_controller/follow_joint_trajectory",
            list(GRIPPER_JOINT_NAMES),
            apply=self._apply_gripper_width,
            read=self._read_gripper_joints,
            lock=self._lock,
            stop_event=self._stop_event,
        )

        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()

        self._pub_timer = self.create_timer(self._pub_period, self._publish_feedback)

        self.get_logger().info(f"Nero MuJoCo bridge ready, model: {model_path}")

    # ── TF helpers ──────────────────────────────────────────────────────────
    def _make_static_tf(self) -> TransformStamped:
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return t

    def _relative_pose(self, body_id):
        """Body pose in its parent's frame via MuJoCo quaternion helpers."""
        xpos = self._data.xpos[body_id]
        xquat = self._data.xquat[body_id]
        parent_id = self._model.body_parentid[body_id]
        if parent_id < 0:
            return xpos.copy(), xquat.copy()

        ppos = self._data.xpos[parent_id]
        pquat = self._data.xquat[parent_id]

        neg = np.zeros(4)
        mujoco.mju_negQuat(neg, pquat)

        diff = np.zeros(3)
        mujoco.mju_sub(diff, xpos, ppos)
        rel_pos = np.zeros(3)
        mujoco.mju_rotVecQuat(rel_pos, diff, neg)

        rel_quat = np.zeros(4)
        mujoco.mju_mulQuat(rel_quat, neg, xquat)
        return rel_pos, rel_quat

    def _body_tf_msg(self, body_name: str) -> TransformStamped | None:
        body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return None
        parent_id = self._model.body_parentid[body_id]
        parent_name = (
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
            if parent_id >= 0 else "world"
        )
        pos, quat = self._relative_pose(body_id)
        t = TransformStamped()
        t.header.frame_id = parent_name
        t.child_frame_id = body_name
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation = _ros_quat(quat)
        return t

    # ── command handlers ────────────────────────────────────────────────────
    def _joint_command_callback(self, msg: JointState):
        if not self.control_enabled:
            self.get_logger().warn("control gate closed, ignoring joint command")
            return
        joint_pos = dict(zip(msg.name, msg.position))
        arm_targets = [joint_pos.get(name) for name in ARM_JOINT_NAMES]
        with self._lock:
            if all(v is not None for v in arm_targets):
                self.runtime.move_j(arm_targets)
            elif any(v is not None for v in arm_targets):
                self.get_logger().warn(
                    "partial arm joint set, expected all of %s", list(ARM_JOINT_NAMES)
                )
            if GRIPPER_JOINT_NAME in joint_pos:
                self.runtime.control_gripper(abs(joint_pos[GRIPPER_JOINT_NAME]))

    def _move_home_callback(self, request, response):
        with self._lock:
            self.runtime.move_home()
        self.get_logger().info("moved to home position")
        return response

    def _emergency_stop_callback(self, request, response):
        with self._lock:
            self.runtime.emergency_stop()
        self.control_enabled = False
        self.get_logger().info("emergency stop triggered, control gate closed")
        return response

    def _control_enable_callback(self, request, response):
        self.control_enabled = request.data
        response.success = True
        response.message = f"control gate {'opened' if request.data else 'closed'}"
        self.get_logger().info(response.message)
        return response

    # ── MoveIt trajectory helpers ────────────────────────────────────────────
    def _read_arm_positions(self):
        """Measured joint1..joint7 positions in ARM_JOINT_NAMES order."""
        return list(self.runtime.state()["arm"]["positions"])

    def _apply_gripper_width(self, positions):
        """Fold gripper_joint1/gripper_joint2 (mirrored) into one finger width."""
        width = positions[0] - positions[1]
        self.runtime.control_gripper(abs(width))

    def _read_gripper_joints(self):
        """Measured gripper opening expressed as [gripper_joint1, gripper_joint2]."""
        width = self.runtime.state()["gripper"]["width"]
        return [width / 2.0, -width / 2.0]

    # ── feedback publication ────────────────────────────────────────────────
    def _publish_feedback(self):
        now = self.get_clock().now().to_msg()

        with self._lock:
            state = self.runtime.state()
            tcp_pos, tcp_quat = self.runtime.tcp_pose()
            if self._object_body_id >= 0:
                obj_pos = self._data.xpos[self._object_body_id].copy()
                obj_quat = self._data.xquat[self._object_body_id].copy()
            else:
                obj_pos = obj_quat = None

        # /joint_states
        js = JointState()
        js.header.stamp = now
        js.name = list(state["arm"]["names"]) + [GRIPPER_JOINT_NAME]
        js.position = list(state["arm"]["positions"]) + [state["gripper"]["width"]]
        js.velocity = list(state["arm"]["velocities"]) + [0.0]
        js.effort = list(state["arm"]["efforts"]) + [state["gripper"]["force"]]
        self._joint_states_pub.publish(js)

        # /feedback/tcp_pose (geometry_msgs/Pose, frame: base_link)
        pose = Pose()
        pose.position = Point(x=float(tcp_pos[0]), y=float(tcp_pos[1]), z=float(tcp_pos[2]))
        pose.orientation = _ros_quat(tcp_quat)
        self._tcp_pose_pub.publish(pose)

        # /feedback/object_pose (geometry_msgs/PoseStamped, frame: world).
        # world == base_link here (fixed base at the origin). Skipped when the
        # loaded model has no grasp object.
        if obj_pos is not None:
            obj_pose = PoseStamped()
            obj_pose.header.stamp = now
            obj_pose.header.frame_id = "world"
            obj_pose.pose.position = Point(
                x=float(obj_pos[0]), y=float(obj_pos[1]), z=float(obj_pos[2])
            )
            obj_pose.pose.orientation = _ros_quat(obj_quat)
            self._object_pose_pub.publish(obj_pose)

        # /clock
        self._clock_pub.publish(Clock(clock=now))

        # /tf
        tf_msg = TFMessage()
        tf_msg.transforms.append(self._static_tf)
        for name in ARM_BODY_NAMES + FINGER_BODY_NAMES:
            t = self._body_tf_msg(name)
            if t is not None:
                t.header.stamp = now
                tf_msg.transforms.append(t)
        self._tf_pub.publish(tf_msg)

    # ── sim loop ────────────────────────────────────────────────────────────
    def _sim_loop(self):
        while not self._stop_event.is_set():
            if self._viewer is not None and not self._viewer.is_running():
                self._stop_event.set()
                break
            with self._lock:
                self.runtime.step()
                if self._viewer is not None:
                    self._viewer.sync()
            self._stop_event.wait(self._sim_period)

    def destroy_node(self):
        self._stop_event.set()
        self._sim_thread.join(timeout=1.0)
        if self._viewer is not None:
            self._viewer.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=None,
                        help="Path to a MuJoCo scene XML (default: resolved from assets/robots"
                             " + the default scene in assets/scenes)")
    parser.add_argument("--viewer", action="store_true",
                        help="Open a mujoco.viewer window showing the arm")
    parsed, ros_args = parser.parse_known_args(args)

    if parsed.model is None:
        model_path = build_scene()["model_path"]
    else:
        model_path = parsed.model

    rclpy.init(args=ros_args)
    node = NeroBridgeNode(model_path, use_viewer=parsed.viewer)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT (Ctrl-C) and SIGTERM (sim/stop.sh) both surface here; the
        # context is already shut down in the SIGTERM case, so don't call
        # rclpy.shutdown() a second time below.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
