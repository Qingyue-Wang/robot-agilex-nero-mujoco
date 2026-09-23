"""Pure geometry helpers for the top-down pick sequence on the Nero arm.

No rclpy / ROS message imports here — this module builds plain (x, y, z, quat)
IK targets and joint-angle seeds; callers (the primitive's `pick` MCP tool,
`moveit/pick_demo.py`) turn those into `moveit_msgs/GetPositionIK` requests
and execute the resulting joint solutions via FollowJointTrajectory. Keeping
this ROS-free makes the geometry testable without a live bridge/move_group.

Why top-down and not a horizontal reach: this arm's wrist "palm" collision
mesh (gripper_base, on link7) is large enough that a horizontal approach lets
the palm reach the object before the fingertips close on it, pushing the cube
instead of grasping it. A vertical approach — flange z-axis pointing straight
down (0, 0, -1) — keeps the palm above the object at every step (verified by
IK + a MuJoCo self-collision / palm-vs-object-bbox check against this scene's
default object pose, then by running the full sequence in the native runtime
and confirming the object lifts off the table).

Two geometry facts specific to this arm/gripper are baked into the constants
below rather than re-derived per call:

  - FLANGE_TO_FINGERTIP: `gripper_flange` is not the fingertip. For a
    straight-down approach the finger-midpoint sits FLANGE_TO_FINGERTIP below
    the flange along the approach (-z) axis, so a target "fingertip height"
    is converted to an IK target of `fingertip_z + FLANGE_TO_FINGERTIP`.
  - IK_SEED / UP_HINT: `compute_ik` (KDL) has many solutions for a
    straight-down orientation; this seed+hint combination consistently lands
    on the joint1=joint3=joint5=joint6=0 branch (only joint2/joint4/joint7
    move), which is collision-free and keeps the palm clear of the object for
    this scene's object position. The arm's *current* pose is NOT a safe seed
    for compute_ik — it can be sitting anywhere, and seeding from an arbitrary
    pose can land on a self-colliding or palm-overlapping branch instead.

Grasp stability is provided by the runtime, not this module. At the grasp the
cube is pinched on the finger's back face as a narrow edge contact (the grooved
front pad sits ~60 mm above it), which lets the cube roll about the pinch axis
during the lift. The native runtime adds a flat, collision-only pad on that
face (see sim/native/runtime.py FINGER_PAD_*) to turn it into a face-to-face
grip whose contact width resists the roll.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))

FLANGE_TO_FINGERTIP = 0.1423  # m, along the approach (-z) axis
UP_HINT = (-1.0, 0.0, 0.0)
GRIPPER_OPEN_WIDTH = 0.08  # m; well clear of a 4 cm cube (gripper max is 0.1)

# Fixed, verified-safe seed for the FIRST compute_ik call of a sequence (see
# module docstring — never seed from the arm's live pose).
IK_SEED = {"joint1": 0.0, "joint2": 0.6, "joint3": 0.0, "joint4": 1.0,
           "joint5": 0.0, "joint6": 0.0, "joint7": 0.0}

# Fingertip heights (metres, added to the object's resting z) for each phase
# of the sequence, in order.
APPROACH_CLEARANCE = 0.14
DESCEND_CLEARANCE = 0.01
LIFT_CLEARANCE = 0.18


def _rotmat_to_quat(rot: np.ndarray) -> tuple[float, float, float, float]:
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


def quat_from_z_axis(target_z: Sequence[float],
                      up_hint: Sequence[float] = UP_HINT) -> tuple[float, float, float, float]:
    """Quaternion whose local z-axis maps to world `target_z`, x roughly
    toward `up_hint` (projected orthogonal to z). Returns (x, y, z, w)."""
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


def straight_down_quat() -> tuple[float, float, float, float]:
    """Quaternion for the flange pointing straight down (approach = -z)."""
    return quat_from_z_axis((0.0, 0.0, -1.0))


def flange_target(obj_x: float, obj_y: float, fingertip_z: float) -> tuple[float, float, float]:
    """(x, y, flange_z) IK target for a given desired fingertip height."""
    return (obj_x, obj_y, fingertip_z + FLANGE_TO_FINGERTIP)


def sequence_targets(obj_x: float, obj_y: float, obj_z: float) -> dict[str, tuple[float, float, float]]:
    """Flange (x, y, z) targets for the approach/descend/lift waypoints, in
    order, given the object's resting position."""
    return {
        "approach": flange_target(obj_x, obj_y, obj_z + APPROACH_CLEARANCE),
        "descend": flange_target(obj_x, obj_y, obj_z + DESCEND_CLEARANCE),
        "lift": flange_target(obj_x, obj_y, obj_z + LIFT_CLEARANCE),
    }
