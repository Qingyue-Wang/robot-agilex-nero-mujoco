#!/usr/bin/env python3
"""Headless top-down pick demo for the simulated AgileX Nero arm + tabletop scene.

Drives MoveIt's standard ROS 2 interfaces directly (no moveit_py /
moveit_commander, same as cartesian_demo.py):

  1. /compute_ik (moveit_msgs/srv/GetPositionIK) — solve joint angles for a
     sequence of `gripper_flange` target poses (all pointing straight down).
  2. /arm_controller/follow_joint_trajectory,
     /gripper_controller/follow_joint_trajectory — execute each arm/gripper
     move through the bridge's standard MoveIt execution interface.

Sequence: approach (above the object, gripper open) -> descend (fingers
straddle the object, gripper open) -> close (grip the object) -> lift (raise
the arm, gripper still closed).

Why top-down and not a horizontal reach: this arm's wrist "palm" collision
mesh (gripper_base, on link7) is large enough that a horizontal approach lets
the palm reach the object before the fingertips close on it, pushing the cube
instead of grasping it (documented in assets/scenes/tabletop_pick.xml and the
main README's tabletop-pick section). A vertical approach — flange z-axis
pointing straight down (0, 0, -1) — keeps the palm above the object at every
step; verified against this scene's default object pose (x=-0.4, y=0, z=0.02)
by IK + a MuJoCo self-collision / palm-vs-object-bbox check, then by running
the full sequence in the native runtime and confirming the object lifts off
the table (see the "Pick demo" section of moveit/README.md).

Two geometry facts specific to this arm/gripper, baked into the constants
below rather than hardcoded per waypoint:

  - FLANGE_TO_FINGERTIP: `gripper_flange` is not the fingertip. For a
    straight-down approach the finger-midpoint sits FLANGE_TO_FINGERTIP below
    the flange along the approach (-z) axis, so a target "fingertip height"
    is converted to an IK target of `fingertip_z + FLANGE_TO_FINGERTIP`.
  - UP_HINT=(-1,0,0): `compute_ik` (KDL) has many solutions for a
    straight-down orientation; this hint consistently lands on the
    joint1=joint3=joint5=joint6=0 branch (only joint2/joint4/joint7 move),
    which is collision-free and keeps the palm clear of the object for this
    scene's object position. Other hints can land on branches with a
    self-colliding wrist fold (see the module docstring's "why top-down" note
    and moveit/README.md).

Prerequisites (all headless):
  - the MuJoCo bridge (default scene = tabletop_pick):  bash sim/start.sh
  - move_group:  ros2 launch nero_gripper_moveit_config move_group.launch.py

Usage:
  python3 pick_demo.py [x y] [--object-z Z]
    x, y: object position in the base_link/world frame (default: this
    scene's default object, -0.4 0.0). --object-z overrides the object's
    resting height (default 0.02, matching the scene's 4 cm cube).

Run inside the Humble environment with the system python3 (ROS 2 Humble is
built for Python 3.10):  export PATH=/usr/bin:$PATH; source /opt/ros/humble/setup.zsh
"""

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory as FollowJT
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
GRIPPER_JOINTS = ["gripper_joint1", "gripper_joint2"]
ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"
IK_SRV = "/compute_ik"
PLANNING_FRAME = "base_link"  # world == base_link (fixed identity offset)
IK_LINK = "gripper_flange"

# Fixed geometry facts for this arm/gripper (see module docstring).
FLANGE_TO_FINGERTIP = 0.1423  # m, along the approach (-z) axis
UP_HINT = (-1.0, 0.0, 0.0)
GRIPPER_OPEN_WIDTH = 0.08  # m; well clear of a 4 cm cube (max is 0.1)
SETTLE_TIME_S = 6.0  # retuned servos settle in ~2-5s; see runtime.py

# KDL is a numerical IK solver with many solutions for a straight-down
# orientation; it converges to whichever one is nearest the seed. The arm's
# *current* pose is NOT a safe seed — the sim can be sitting anywhere (mid a
# previous move, or wherever `move_home` last left it), and seeding from an
# arbitrary pose can land on a self-colliding or palm-overlapping branch
# instead of the verified one (see the module docstring). Always seed the
# first waypoint from this fixed, bent-elbow pose instead.
IK_SEED = {"joint1": 0.0, "joint2": 0.6, "joint3": 0.0, "joint4": 1.0,
           "joint5": 0.0, "joint6": 0.0, "joint7": 0.0}


def _rotmat_to_quat(rot):
    """3x3 rotation matrix -> quaternion (x, y, z, w)."""
    tr = np.trace(rot)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def quat_from_z_axis(target_z, up_hint=UP_HINT):
    """Quaternion whose local z-axis maps to world `target_z`, x roughly
    toward `up_hint` (projected orthogonal to z)."""
    z = np.array(target_z, dtype=float)
    z /= np.linalg.norm(z)
    x_hint = np.array(up_hint, dtype=float)
    x = x_hint - np.dot(x_hint, z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0.0, 1.0, 0.0])
        x = x - np.dot(x, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return _rotmat_to_quat(np.column_stack([x, y, z]))


class PickDemo(Node):
    def __init__(self, obj_x, obj_y, obj_z):
        super().__init__("pick_demo")
        self.obj_x, self.obj_y, self.obj_z = obj_x, obj_y, obj_z
        self._arm_joints = None
        self._gripper_width = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self._ik = self.create_client(GetPositionIK, IK_SRV)
        self._arm_act = ActionClient(self, FollowJT, ARM_ACTION)
        self._gripper_act = ActionClient(self, FollowJT, GRIPPER_ACTION)

    def _js_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if set(ARM_JOINTS) <= set(pos):
            self._arm_joints = {n: pos[n] for n in ARM_JOINTS}
        if "gripper" in pos:
            self._gripper_width = pos["gripper"]

    def wait_ready(self, timeout_s=15.0):
        for client, name in ((self._ik, IK_SRV), (self._arm_act, ARM_ACTION), (self._gripper_act, GRIPPER_ACTION)):
            ok = client.wait_for_server(timeout_sec=timeout_s) if hasattr(client, "wait_for_server") \
                else client.wait_for_service(timeout_sec=timeout_s)
            if not ok:
                self.get_logger().error(f"{name} not available")
                return False
        t0 = time.time()
        while self._arm_joints is None and time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._arm_joints is None:
            self.get_logger().error("no /joint_states for joint1..7 (is the bridge up?)")
            return False
        return True

    def solve_ik(self, fingertip_z, seed):
        """IK for the flange pointing straight down, fingertip at `fingertip_z`."""
        flange_z = fingertip_z + FLANGE_TO_FINGERTIP
        quat = quat_from_z_axis((0, 0, -1))

        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        rs = RobotState()
        rs.joint_state.name = list(seed.keys())
        rs.joint_state.position = list(seed.values())
        req.ik_request.robot_state = rs
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 3
        pose = PoseStamped()
        pose.header.frame_id = PLANNING_FRAME
        pose.pose.position.x = self.obj_x
        pose.pose.position.y = self.obj_y
        pose.pose.position.z = flange_z
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quat
        req.ik_request.pose_stamped = pose
        req.ik_request.ik_link_name = IK_LINK

        fut = self._ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None or resp.error_code.val != 1:
            code = None if resp is None else resp.error_code.val
            self.get_logger().error(f"compute_ik failed for fingertip_z={fingertip_z}: error_code={code}")
            return None
        return {n: p for n, p in zip(resp.solution.joint_state.name, resp.solution.joint_state.position)
                if n in ARM_JOINTS}

    def _execute(self, action_client, joint_names, positions, seconds, wait_settle):
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=int(seconds), nanosec=0)
        traj.points = [pt]
        goal = FollowJT.Goal()
        goal.trajectory = traj
        send = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected")
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        r = res.result().result
        ok = r.error_code == FollowJT.Result.SUCCESSFUL
        if not ok:
            self.get_logger().error(f"execute failed: error_code={r.error_code} {r.error_string}")
            return ok
        # The bridge reports success once the point is *commanded*, not once
        # physically settled (retuned servos, but still not instantaneous —
        # see runtime.py's "Servo retune"). Give it time before the next move.
        if wait_settle:
            deadline = time.time() + SETTLE_TIME_S
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        return ok

    def move_arm(self, joint_targets, label, wait_settle=True):
        self.get_logger().info(f"{label}: {joint_targets}")
        positions = [joint_targets[n] for n in ARM_JOINTS]
        return self._execute(self._arm_act, ARM_JOINTS, positions, 2, wait_settle)

    def move_gripper(self, width, label, wait_settle=True):
        """`width` here is the ARM_JOINTS-style single opening (metres); folded
        into the mirrored gripper_joint1/gripper_joint2 pair the action expects."""
        self.get_logger().info(f"{label}: width={width:.3f}")
        half = width / 2.0
        return self._execute(self._gripper_act, GRIPPER_JOINTS, [half, -half], 2, wait_settle)

    def run(self):
        if not self.wait_ready():
            return 1

        # Fingertip target heights: comfortably above the object, straddling
        # it just below its top face (4 cm cube -> top at obj_z + 0.02), then
        # well clear on lift.
        approach_sol = self.solve_ik(self.obj_z + 0.14, IK_SEED)
        if approach_sol is None:
            return 1
        descend_sol = self.solve_ik(self.obj_z + 0.01, approach_sol)
        if descend_sol is None:
            return 1
        lift_sol = self.solve_ik(self.obj_z + 0.18, descend_sol)
        if lift_sol is None:
            return 1

        if not self.move_arm(approach_sol, "1/4 approach (gripper open)"):
            return 1
        if not self.move_gripper(GRIPPER_OPEN_WIDTH, "   opening gripper", wait_settle=False):
            return 1
        if not self.move_arm(descend_sol, "2/4 descend"):
            return 1
        if not self.move_gripper(0.0, "3/4 closing gripper"):
            return 1
        if not self.move_arm(lift_sol, "4/4 lift"):
            return 1

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        width = self._gripper_width if self._gripper_width is not None else -1.0
        self.get_logger().info(f"done. final gripper width: {width:.4f} m "
                               f"(read /feedback/object_pose to confirm the object lifted)")
        return 0


def main():
    rclpy.init()
    args = sys.argv[1:]
    obj_z = 0.02
    if "--object-z" in args:
        i = args.index("--object-z")
        obj_z = float(args[i + 1])
        del args[i:i + 2]
    obj_x, obj_y = (-0.4, 0.0)
    if len(args) >= 2:
        obj_x, obj_y = float(args[0]), float(args[1])

    node = PickDemo(obj_x, obj_y, obj_z)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
