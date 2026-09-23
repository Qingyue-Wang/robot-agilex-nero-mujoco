"""Native MuJoCo control adapter for the Agilex Nero arm.

Mirrors the command/feedback semantics of the real ROS 2 driver
(agx_arm_ctrl_single_node.py in agx_arm_ros) so the same joint names,
units and command shapes work against either the real arm or this
simulated one. Exposes command()/step()/state()/reset() as described
in the MuJoCo-to-Robonix onboarding guide's "Native 控制适配器" section.

This is a self-contained copy of the upstream controller so the body
package does not depend on the tutorial repository at runtime.
"""

import logging
import time

import mujoco
import numpy as np

logger = logging.getLogger(__name__)

ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
GRIPPER_ACTUATOR_NAME = "gripper"
GRIPPER_DRIVE_JOINT_NAME = "gripper_joint1"
GRIPPER_JOINT_NAME = "gripper"  # pseudo-joint exposed over ROS

# Matches AgxGripperWrapper.WIDTH_MIN/WIDTH_MAX in agx_arm_ros (unit: m).
# The two fingers open symmetrically, so real-world width == 2x the drive
# joint's travel; GRIPPER_WIDTH_MAX is validated against the model at init
# instead of assuming the MJCF range, in case the model changes.
GRIPPER_WIDTH_MIN = 0.0
GRIPPER_WIDTH_MAX = 0.1

HOME_KEYFRAME_NAME = "home"


class UnreachableCommandError(ValueError):
    """Raised when a command cannot be mapped onto this model at all."""


class NeroMujocoController:
    """Thin adapter between Bridge-style commands and a Nero MjModel/MjData.

    Command timestamps are tracked so a caller can detect a stale link
    (see is_link_stale()); since all actuators here are position
    controllers, holding the last commanded target is already the safe
    behavior while a link is stale, so step() does not need to alter ctrl.
    """

    def __init__(self, model, data, command_timeout=0.5):
        self.model = model
        self.data = data
        self.command_timeout = command_timeout

        self._arm_actuator_ids = self._resolve_actuator_ids(ARM_JOINT_NAMES)
        self._gripper_actuator_id = self._resolve_actuator_ids([GRIPPER_ACTUATOR_NAME])[0]
        self._arm_joint_ids = self._resolve_joint_ids(ARM_JOINT_NAMES)
        self._gripper_joint_id = self._resolve_joint_ids([GRIPPER_DRIVE_JOINT_NAME])[0]

        gripper_ctrlrange = model.actuator_ctrlrange[self._gripper_actuator_id]
        self._finger_travel_max = float(gripper_ctrlrange[1])

        self._home_keyframe_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_KEY, HOME_KEYFRAME_NAME
        )

        self._last_command_time = None
        self._stale_link_logged = False

        # Hold the pose the model starts at until the first command arrives,
        # instead of snapping to whatever ctrl happens to default to.
        self._sync_ctrl_to_current_qpos()

    def _resolve_actuator_ids(self, names):
        ids = []
        for name in names:
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if actuator_id < 0:
                raise UnreachableCommandError(f"actuator '{name}' not found in model")
            ids.append(actuator_id)
        return ids

    def _resolve_joint_ids(self, names):
        ids = []
        for name in names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise UnreachableCommandError(f"joint '{name}' not found in model")
            ids.append(joint_id)
        return ids

    def _sync_ctrl_to_current_qpos(self):
        qpos_addrs = [self.model.jnt_qposadr[j] for j in self._arm_joint_ids]
        self.data.ctrl[self._arm_actuator_ids] = self.data.qpos[qpos_addrs]
        gripper_qpos_addr = self.model.jnt_qposadr[self._gripper_joint_id]
        self.data.ctrl[self._gripper_actuator_id] = self.data.qpos[gripper_qpos_addr]

    def _mark_commanded(self):
        self._last_command_time = time.monotonic()
        self._stale_link_logged = False

    def _clip_to_actuator_range(self, actuator_id, value):
        low, high = self.model.actuator_ctrlrange[actuator_id]
        clipped = float(np.clip(value, low, high))
        if clipped != value:
            logger.warning(
                "actuator %s: commanded %.4f out of range [%.4f, %.4f], clipped to %.4f",
                self.model.actuator(actuator_id).name, value, low, high, clipped,
            )
        return clipped

    def move_j(self, joints):
        """Position command for joint1..joint7, unit: rad. Matches move_j in agx_arm_ctrl."""
        if len(joints) != len(ARM_JOINT_NAMES):
            raise UnreachableCommandError(
                f"move_j expects {len(ARM_JOINT_NAMES)} joint values, got {len(joints)}"
            )
        for actuator_id, target in zip(self._arm_actuator_ids, joints):
            self.data.ctrl[actuator_id] = self._clip_to_actuator_range(actuator_id, target)
        self._mark_commanded()

    def move_home(self):
        """Matches the move_home service: send the arm to its home pose (a
        bent-elbow pose, not the raw all-zero straight-up pose — the zero
        pose sits near a kinematic singularity, same as noted in
        moveit/README.md's Cartesian demo)."""
        self.move_j([0.0, 0.4, 0.0, 0.2, 0.0, 0.0, 0.0])

    def control_gripper(self, width, force=None):
        """Width in meters (0..GRIPPER_WIDTH_MAX), matches AgxGripperWrapper.move()."""
        if not (GRIPPER_WIDTH_MIN <= width <= GRIPPER_WIDTH_MAX):
            raise UnreachableCommandError(
                f"gripper width must be in [{GRIPPER_WIDTH_MIN}, {GRIPPER_WIDTH_MAX}], got {width}"
            )
        finger_travel = width / 2.0 * (self._finger_travel_max / (GRIPPER_WIDTH_MAX / 2.0))
        self.data.ctrl[self._gripper_actuator_id] = self._clip_to_actuator_range(
            self._gripper_actuator_id, finger_travel
        )
        self._mark_commanded()

    def emergency_stop(self):
        """Matches emergency_stop service: hold the current pose, don't zero it out."""
        self._sync_ctrl_to_current_qpos()
        self._mark_commanded()
        logger.info("emergency stop: holding current pose")

    def reset(self):
        """Deterministic reset to the 'home' keyframe (falls back to zero pose)."""
        if self._home_keyframe_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self._home_keyframe_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
        self._sync_ctrl_to_current_qpos()
        self._last_command_time = None
        self._stale_link_logged = False

    def command(self, message):
        """Dispatch a Bridge-style command dict. Unreachable ones are rejected, not crashed on."""
        command_type = message.get("type")
        try:
            if command_type == "move_j":
                self.move_j(message["joints"])
            elif command_type == "move_home":
                self.move_home()
            elif command_type == "gripper":
                self.control_gripper(message["width"], message.get("force"))
            elif command_type == "emergency_stop":
                self.emergency_stop()
            elif command_type == "reset":
                self.reset()
            else:
                raise UnreachableCommandError(f"unknown command type '{command_type}'")
        except UnreachableCommandError as exc:
            logger.error("rejected command %r: %s", message, exc)

    def is_link_stale(self):
        if self._last_command_time is None:
            return False
        return (time.monotonic() - self._last_command_time) > self.command_timeout

    def step(self):
        """Call once per control tick before mj_step. Position actuators hold their
        last target on their own, so a stale link only needs a one-time warning."""
        if self.is_link_stale() and not self._stale_link_logged:
            logger.warning("no command received for over %.2fs, holding last pose", self.command_timeout)
            self._stale_link_logged = True

    def state(self):
        """Feedback matching feedback/joint_states + feedback/gripper_status semantics."""
        qpos_addrs = [self.model.jnt_qposadr[j] for j in self._arm_joint_ids]
        qvel_addrs = [self.model.jnt_dofadr[j] for j in self._arm_joint_ids]
        arm_positions = self.data.qpos[qpos_addrs].tolist()
        arm_velocities = self.data.qvel[qvel_addrs].tolist()
        arm_efforts = self.data.actuator_force[self._arm_actuator_ids].tolist()

        gripper_qpos_addr = self.model.jnt_qposadr[self._gripper_joint_id]
        finger_travel = float(self.data.qpos[gripper_qpos_addr])
        gripper_width = finger_travel * 2.0 * (GRIPPER_WIDTH_MAX / 2.0 / self._finger_travel_max)

        return {
            "arm": {
                "names": list(ARM_JOINT_NAMES),
                "positions": arm_positions,
                "velocities": arm_velocities,
                "efforts": arm_efforts,
                "endPose": None,
            },
            "gripper": {
                "width": gripper_width,
                "force": float(self.data.actuator_force[self._gripper_actuator_id]),
            },
        }
