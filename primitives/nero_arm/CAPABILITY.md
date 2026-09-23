---
description: Control the simulated fixed-base AgileX Nero 7-DOF arm and parallel gripper over ROS 2 joint space.
---

# Nero arm (MuJoCo)

The `nero_arm` primitive is the Robonix boot-layer wrapper around the Nero
MuJoCo simulation (`sim/bridge/bridge_node.py`). It owns the sim lifecycle:
`on_activate` spawns `sim/start.sh`, waits for the first `/joint_states` as
proof the bridge is up, then declares the arm's ROS 2 topics on Atlas. The
sim process is killed on deactivate / shutdown.

## Topics

- `joint_states` (`sensor_msgs/JointState`, topic_out `/joint_states`) — measured
  feedback, names `joint1`..`joint7` in radians plus `gripper` (finger opening
  in metres, 0.0..0.1).
- `joint_command` (`sensor_msgs/JointState`, topic_in `/joint_command`) — named
  joint target; a `gripper` entry commands the finger opening in metres.
- `end_pose` (`geometry_msgs/Pose`, topic_out `/feedback/tcp_pose`) — end-effector
  pose (finger midpoint, oriented like link7) in the `base_link` frame.
- `object_pose` (`geometry_msgs/PoseStamped`, topic_out `/feedback/object_pose`) —
  grasp-target object pose in the `world` (== `base_link`) frame; published only
  when the sim loads a scene with an `object` body (the default tabletop scene).

## RPC tools (MCP)

- `status` — read connection/enable/joint/gripper state without moving.
- `set_enabled` — open/close the bridge control gate (closed gate ignores commands).
- `emergency_stop` — hold the current pose and close the control gate.
- `move_home` — command the arm to its home pose (a bent-elbow pose, not the
  raw all-zero straight pose — see `NeroMujocoController.move_home`). Also
  called automatically once on activation unless `move_home_on_activate:
  false` is set in the primitive's config.
- `follow_joint_trajectory` — execute a timed joint trajectory through the
  bridge's FollowJointTrajectory action (the standard MoveIt execution
  interface). `trajectory.joint_names` selects the controller: `joint1`..`joint7`
  → the arm controller, `gripper_joint1`/`gripper_joint2` → the gripper
  controller. The goal reports success once the final point is *commanded*;
  read `joint_states` for the true (settled) pose.

## Trajectory / MoveIt interface

The bridge exposes the standard MoveIt execution interface as two
`control_msgs/FollowJointTrajectory` actions:

- `/arm_controller/follow_joint_trajectory` — `joint1`..`joint7` (radians).
- `/gripper_controller/follow_joint_trajectory` — `gripper_joint1` /
  `gripper_joint2` (folded into a single finger width).

`follow_joint_trajectory` is the primitive-level wrapper around these actions;
`move_group` can drive the same actions directly over ROS 2, so a MoveIt graph
drives this sim exactly as it would a ros2_control controller.

Joint commands are low-level; there is no Cartesian/IK stage yet (Phase 3).
The sim retunes the vendored (overdamped) servo gains on load — see the main
README's "Servo retune" section — so a commanded pose settles in a few
seconds; still allow that settling time before reading `end_pose`.
