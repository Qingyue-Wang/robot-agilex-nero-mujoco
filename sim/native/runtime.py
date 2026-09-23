"""Native MuJoCo runtime for the Nero arm.

Holds MjModel/MjData, steps physics, and drives the vendored controller
each tick. The ROS 2 boundary lives in `sim/bridge/bridge_node.py`, which
imports this runtime and calls `step()`/`state()` on its own cadence; the
controller is exercised in-process (no WebSocket hop) because this body
package is Native-only, matching the onboarding guide's note that
in-process invocation is simpler when the vendor controller runs in the
same dependency environment.
"""

from __future__ import annotations

import logging

import mujoco
import numpy as np

from .controller import NeroMujocoController

logger = logging.getLogger(__name__)

# Body frames published as TF / used for TCP estimation. The upstream MJCF
# has no `gripper_flange`/`gripper_base` body (those are geoms inside link7),
# so the end-effector is reported as the midpoint of the two finger bodies.
ARM_BODY_NAMES = ("link1", "link2", "link3", "link4", "link5", "link6", "link7")
FINGER_BODY_NAMES = ("gripper_link1", "gripper_link2")


# Arm-joint servo retune, applied on top of the vendored values (never edit
# vendor/nero_arm.xml in place — see UPSTREAM.md). The vendored gains
# (damping=2000, kp=10..80 per joint) are so overdamped that a commanded move
# takes tens to hundreds of seconds to arrive, AND still settles ~20% short of
# the target — a plain PD position servo has no integral term, so gravity
# torque leaves a permanent steady-state error of roughly (gravity_torque/kp)
# regardless of damping. Measured on this model: at the vendored gains, a
# joint2/joint4 move to (0.4, 0.2) rad is still ~50% short after 8s and only
# reaches the *same* ~20% steady-state offset as these new gains after several
# minutes. Scaling kp/kv by ARM_KP_SCALE while cutting damping to
# ARM_DAMPING brings settle time down to ~2-5s with <5% steady-state error on
# the heaviest joints, verified with multi-joint collision-free moves without
# the actuators saturating in steady state.
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
ARM_DAMPING = 100.0
ARM_KP_SCALE = 5.0
ARM_KV_SCALE = 5.0

# The wrist joints (joint5/joint6/joint7) ship with 4-8x lower gains than the
# shoulder/elbow (vendor kp=10 vs 40-80), yet they carry the gripper at the end
# of the arm. At the base ARM_KP_SCALE the wrist-roll joint7 is still ~0.23 rad
# (13°) short of a commanded descend pose after 4s — enough to rotate the
# gripper's closing axis so a top-down grasp pushes the cube aside instead of
# holding it. This extra multiplier (applied on top of ARM_KP_SCALE/ARM_KV_SCALE)
# brings the wrist's settle time in line with the rest of the arm (~2-4s) without
# saturating its +-100 actuator in steady state (max ~86 during a descend move).
WRIST_JOINTS = ("joint5", "joint6", "joint7")
WRIST_KP_SCALE = 2.0

# The gripper's own actuator forcerange is 10x smaller than the arm's
# (+-10 vs +-100), so it needs its own (much smaller) damping value rather
# than reusing ARM_DAMPING — the same measurement process (drive to a
# mid-range width, watch qpos vs. actuator saturation) landed on these.
GRIPPER_DAMPING = 20.0
GRIPPER_KP = 400.0
GRIPPER_KV = 25.0

# Flat contact pad on each finger's grasping face (see _load_model). The
# vendored finger STL is a thin paddle; at the grasp the 4 cm cube is pinched
# on the finger's *back* face (body -Z, near the palm) rather than the grooved
# front face, so the contact is a narrow edge and the cube slowly rolls ~60°
# about the pinch axis and is ejected during the lift. No friction tweak can
# stop that — the cube is in neutral equilibrium about the pinch axis (COM on
# the axis), so friction only damps the roll, it can't restore it. A thin flat
# pad on that face turns the edge contact into a face-to-face grip: the contact
# *width* itself supplies the restoring torque. The pad is collision-only
# (invisible) and sits FLUSH with the finger's back face — its -Z surface at
# z=0, the rest buried in the 26.5 mm-thick finger. It must NOT stick out
# toward the cube: both fingers' pads point at each other, so even 2 mm of
# stick-out per side makes them collide when the gripper is closed, jamming the
# open (the gripper stalls at ~0.008 m instead of reaching the 0.08 m open).
FINGER_PAD_HALF_SIZE = (0.023, 0.015, 0.002)   # 46 x 30 mm pad, 4 mm thick
FINGER_PAD_POS = (0.0, 0.0, 0.002)             # flush: -Z face at z=0, no stick-out
FINGER_PAD_FRICTION = (1.0, 1.0, 0.5)


def _load_model(model_path: str) -> mujoco.MjModel:
    """Compile a MuJoCo model, tolerating the vendored arm's `home` keyframe,
    and retune the arm/gripper position servos (see the constants above).

    The vendored Nero model ships a `home` keyframe whose `qpos` hardcodes the
    arm-only dof count (9). That is fine for the bare arm, but once a scene
    composes in extra dofs (e.g. a free grasp object) the keyframe no longer
    matches the full `nq` and a plain `from_xml_path` refuses to compile. Load
    via `MjSpec`, drop that keyframe, retune the joints, then compile: the
    controller's `reset()` falls back to `mj_resetData`, which for this arm is
    the same zero/home pose.
    """
    spec = mujoco.MjSpec.from_file(model_path)
    home = spec.key("home")
    if home is not None:
        spec.delete(home)

    for name in ARM_JOINTS:
        spec.joint(name).damping = ARM_DAMPING
        actuator = spec.actuator(name)
        extra = WRIST_KP_SCALE if name in WRIST_JOINTS else 1.0
        kp = actuator.gainprm[0] * ARM_KP_SCALE * extra
        kv = -actuator.biasprm[2] * ARM_KV_SCALE * extra
        actuator.gainprm[0] = kp
        actuator.biasprm[1] = -kp
        actuator.biasprm[2] = -kv

    # Retune BOTH finger joints. gripper_joint2 is equality-slaved to mirror
    # gripper_joint1, so its own (vendor-default) damping still resists the
    # opening motion through the constraint — leaving it at 2000 makes the
    # gripper crawl (a 0.04m->0.08m open took >15s instead of <1s).
    for name in ("gripper_joint1", "gripper_joint2"):
        spec.joint(name).damping = GRIPPER_DAMPING
    gripper_actuator = spec.actuator("gripper")
    gripper_actuator.gainprm[0] = GRIPPER_KP
    gripper_actuator.biasprm[1] = -GRIPPER_KP
    gripper_actuator.biasprm[2] = -GRIPPER_KV

    # Flat contact pad on each finger's grasping face (see FINGER_PAD_* above).
    # Both fingers grasp the cube on their body -Z face (the cube's world-Y
    # width maps onto the finger's Z axis), so the pad is thin in Z (flat face
    # parallel to the finger's inner face) and wide/tall in X/Y, mirroring a
    # real gripper rubber pad. The pad is added to the body, not the mesh, so
    # the STL geometry is left untouched.
    for body_name in ("gripper_link1", "gripper_link2"):
        pad = spec.body(body_name).add_geom(type=mujoco.mjtGeom.mjGEOM_BOX)
        pad.size = FINGER_PAD_HALF_SIZE
        pad.pos = FINGER_PAD_POS
        pad.friction = FINGER_PAD_FRICTION

    return spec.compile()


class NeroRuntime:
    """Physics + controller ownership for a single fixed-base Nero arm."""

    def __init__(self, model_path: str, sim_hz: float = 500.0, command_timeout: float = 0.5):
        self.model_path = model_path
        self.model = _load_model(model_path)
        self.data = mujoco.MjData(self.model)
        self.controller = NeroMujocoController(self.model, self.data, command_timeout)
        self._sim_period = 1.0 / sim_hz

        # Compute forward kinematics once so xpos/xquat (and hence tcp_pose)
        # are valid before the first physics step.
        mujoco.mj_forward(self.model, self.data)

        # Resolve body ids once for TF + TCP publication.
        self._arm_body_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n)
                              for n in ARM_BODY_NAMES]
        self._finger_body_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n)
                                 for n in FINGER_BODY_NAMES]
        for name, body_id in zip(ARM_BODY_NAMES + FINGER_BODY_NAMES,
                                 self._arm_body_ids + self._finger_body_ids):
            if body_id < 0:
                logger.warning("body '%s' not found in model; TF for it will be skipped", name)

    def step(self) -> None:
        """One physics tick: refresh controller, then mj_step."""
        self.controller.step()
        mujoco.mj_step(self.model, self.data)

    def state(self) -> dict:
        """Combined arm + gripper feedback (mirrors controller.state())."""
        return self.controller.state()

    def tcp_pose(self):
        """End-effector pose: midpoint of the two fingers, oriented like link7.

        Returns (position: (3,) np.ndarray, quaternion: (4,) np.ndarray in
        MuJoCo [w,x,y,z] order). Caller converts to the ROS convention.
        """
        link7_id = self._arm_body_ids[6]  # ARM_BODY_NAMES index of link7
        quat = self.data.xquat[link7_id].copy()
        points = [self.data.xpos[b] for b in self._finger_body_ids if b >= 0]
        if not points:
            pos = self.data.xpos[link7_id].copy()
        else:
            pos = np.mean(points, axis=0)
        return pos, quat

    def body_pose(self, name: str):
        """Return (xpos, xquat) for a named body, or None if it doesn't exist."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            return None
        return self.data.xpos[body_id], self.data.xquat[body_id]

    def move_j(self, joints) -> None:
        self.controller.move_j(joints)

    def move_home(self) -> None:
        self.controller.move_home()

    def control_gripper(self, width, force=None) -> None:
        self.controller.control_gripper(width, force)

    def emergency_stop(self) -> None:
        self.controller.emergency_stop()

    def reset(self) -> None:
        self.controller.reset()
        mujoco.mj_forward(self.model, self.data)
