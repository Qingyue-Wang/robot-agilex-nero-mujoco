#!/usr/bin/env python3
"""Headless Cartesian motion demo for the simulated AgileX Nero arm.

This does NOT use moveit_py / moveit_commander (not installed on Humble here).
It drives MoveIt's standard ROS 2 interfaces directly:

  0. (optional) joint-space "ready" move — bends the elbow away from the straight
     (near-singular) home pose via the bridge's FollowJointTrajectory action.
  1. /compute_fk (moveit_msgs/srv/GetPositionFK) — exact current pose of the
     planning tip link (gripper_flange) in the robot model.
  2. /compute_cartesian_path (moveit_msgs/srv/GetCartesianPath) — turn a straight
     Cartesian displacement of the flange into a joint trajectory for "arm".
  3. /arm_controller/follow_joint_trajectory — execute it (standard MoveIt
     execution interface; see config/moveit_controllers.yaml).

Prerequisites (all headless):
  - the MuJoCo bridge:  bash sim/start.sh
  - move_group:         ros2 launch nero_gripper_moveit_config move_group.launch.py

Usage:
  python3 cartesian_demo.py [dx] [dy] [dz] [--no-ready]
    default step is 0.05 m in +x; pass --no-ready to skip the elbow-bend prep
    (a straight-arm pose is near-singular, so upward/outward IK may fail).

Run inside the Humble environment with the system python3 (ROS 2 Humble is
built for Python 3.10):  export PATH=/usr/bin:$PATH; source /opt/ros/humble/setup.zsh
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory as FollowJT
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
ARM_ACTION = "/arm_controller/follow_joint_trajectory"
CARTESIAN_SRV = "/compute_cartesian_path"
FK_SRV = "/compute_fk"
PLANNING_FRAME = "base_link"  # world == base_link (fixed identity offset)


class CartesianDemo(Node):
    def __init__(self, step, do_ready):
        super().__init__("cartesian_demo")
        self.step = step
        self.do_ready = do_ready
        self._joints = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self._fk = self.create_client(GetPositionFK, FK_SRV)
        self._cart = self.create_client(GetCartesianPath, CARTESIAN_SRV)
        self._act = ActionClient(self, FollowJT, ARM_ACTION)

    def _js_cb(self, msg):
        if set(ARM_JOINTS) <= set(msg.name):
            self._joints = {n: p for n, p in zip(msg.name, msg.position)}

    def current_start_state(self):
        rs = RobotState()
        js = JointState()
        js.name = ARM_JOINTS
        js.position = [float(self._joints[n]) for n in ARM_JOINTS]
        rs.joint_state = js
        return rs

    def fk_flange_pose(self):
        req = GetPositionFK.Request()
        req.header.frame_id = PLANNING_FRAME
        req.fk_link_names = ["gripper_flange"]
        req.robot_state = self.current_start_state()
        fut = self._fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
            return None
        return resp.pose_stamped[0].pose

    def plan(self, flange):
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.group_name = "arm"
        req.link_name = "gripper_flange"
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = False
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.start_state = self.current_start_state()

        target = Pose()
        target.position.x = flange.position.x + self.step[0]
        target.position.y = flange.position.y + self.step[1]
        target.position.z = flange.position.z + self.step[2]
        target.orientation = flange.orientation  # keep flange orientation
        req.waypoints = [target]

        self.get_logger().info(f"planning Cartesian step {self.step} from flange "
                               f"({flange.position.x:.3f}, {flange.position.y:.3f}, "
                               f"{flange.position.z:.3f})")
        fut = self._cart.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def execute(self, traj):
        goal = FollowJT.Goal()
        goal.trajectory = traj
        self.get_logger().info(f"sending {len(traj.points)} points to bridge arm_controller")
        send = self._act.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected by bridge arm_controller")
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        r = res.result().result
        ok = r.error_code == FollowJT.Result.SUCCESSFUL
        self.get_logger().info(f"arm_controller result: error_code={r.error_code} {r.error_string}")
        return ok

    def move_ready(self):
        """Bend the elbow away from the straight (singular) home pose."""
        target = {n: float(self._joints[n]) for n in ARM_JOINTS}
        target["joint2"] = 0.4
        target["joint4"] = 0.2
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [target[n] for n in ARM_JOINTS]
        pt.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [pt]
        self.get_logger().info("moving to ready pose (joint2=0.4, joint4=0.2)")
        if not self.execute(traj):
            return False
        # The vendored model is heavily overdamped (damping=2000), so the goal
        # returns once the point is *commanded*; poll until the arm actually
        # settles near the target before planning the Cartesian path.
        want = {"joint2": 0.4, "joint4": 0.2}
        t0 = time.time()
        while time.time() - t0 < 25.0:
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            if all(abs(self._joints[j] - v) < 0.02 for j, v in want.items()):
                return True
        self.get_logger().warn("ready pose did not settle within 25 s; proceeding anyway")
        return True

    def run(self):
        if not self._fk.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("move_group /compute_fk not available")
            return 1
        if not self._cart.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("move_group /compute_cartesian_path not available")
            return 1
        if not self._act.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("bridge arm_controller action not available")
            return 1

        t0 = time.time()
        while self._joints is None and time.time() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joints is None:
            self.get_logger().error("no /joint_states for joint1..7 (is the bridge up?)")
            return 1

        if self.do_ready and not self.move_ready():
            return 1

        start = [float(self._joints[n]) for n in ARM_JOINTS]
        flange = self.fk_flange_pose()
        if flange is None:
            self.get_logger().error("FK failed for gripper_flange")
            return 1

        resp = self.plan(flange)
        if resp is None:
            self.get_logger().error("plan returned None")
            return 1
        if resp.error_code.val != 1:  # MoveItErrorCodes.SUCCESS
            self.get_logger().error(f"plan failed: error_code={resp.error_code.val}")
            return 1
        if resp.fraction < 0.99:
            self.get_logger().warn(f"partial Cartesian path, fraction={resp.fraction:.3f}")
            return 1
        if not resp.solution.joint_trajectory.points:
            self.get_logger().error("plan returned empty trajectory")
            return 1

        self.get_logger().info(f"plan ok: {len(resp.solution.joint_trajectory.points)} points, "
                               f"fraction={resp.fraction:.3f}")
        if not self.execute(resp.solution.joint_trajectory):
            return 1

        time.sleep(3.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        end = [float(self._joints[n]) for n in ARM_JOINTS]
        moved = max(abs(a - b) for a, b in zip(start, end))
        self.get_logger().info(f"max joint displacement after execute: {moved:.4f} rad")
        return 0 if moved > 1e-3 else 2


def main():
    rclpy.init()
    args = sys.argv[1:]
    do_ready = "--no-ready" not in args
    args = [a for a in args if a != "--no-ready"]
    try:
        step = [float(a) for a in args[:3]]
    except ValueError:
        print("usage: cartesian_demo.py [dx dy dz] [--no-ready]", file=sys.stderr)
        return 2
    while len(step) < 3:
        step.append(0.0)
    step = step[:3]

    node = CartesianDemo(step, do_ready)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
