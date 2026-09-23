# robot-agilex-nero-mujoco

Native MuJoCo simulation of the fixed-base **AgileX Nero** 7-DOF arm with a
parallel gripper, exposed to Robonix as an `arm` primitive. This is the
Phase 1+2 implementation of `tutorial_for_mujoco/mujoco-simulation-onboarding.md`
(sim layer + joint-space arm primitive end-to-end). By default it loads a
**tabletop pick scene**: the arm on a worktable with a free-floating grasp
target (a 4 cm cube) whose pose is published for Robonix to observe.

## Layout

```
assets/robots/nero_arm/        MuJoCo model (vendored nero_arm.xml + meshes)
assets/scenes/                 Scene composition (tabletop_pick: floor/table/object/lights)
sim/                           Native sim + ROS 2 bridge (physics loop, topics, services)
primitives/nero_arm/           Robonix arm primitive (boot-layer wrapper + MCP tools)
moveit/                        Self-contained MoveIt config + Cartesian demo
urdf/nero_arm.urdf             Soma-facing kinematic tree (links/joints only)
scripts/setup_venv.sh          Recreates the repo-local .venv (MuJoCo + rclpy)
robonix_manifest.yaml          Deploy manifest (rbnx boot)
soma.yaml                      Body description served by Soma
```

## Scene

`assets/scenes/tabletop_pick.xml` composes the environment around the arm
(via `<include>` of the arm wrapper, keeping `base_link` at the world origin so
the MoveIt/planning frame is unchanged):

- a `floor` plane and a `table` whose top surface sits at `z=0` (the arm base
  rests on it), and
- a free `object` (a 4 cm cube, `mass=0.05`) resting on the table 0.4 m in
  front of the arm, on the **-x side** — at `joint1=0` the shoulder bends the
  arm out toward -x by construction, and `joint1`'s travel (±155°) can't yaw
  a full 180° over to +x. `joint2=0.574, joint4=2.018` (verified by forward
  kinematics) puts the gripper-finger midpoint within a few mm of the object.

The bridge publishes the object's world-frame pose on `/feedback/object_pose`
(`geometry_msgs/PoseStamped`); the primitive declares it as
`robonix/primitive/arm/object_pose`. `assets/scenes/index.json` selects the
default scene; pass `--model <xml>` to `sim/start.sh` to load a different
model (e.g. the bare arm `assets/robots/nero_arm/robot_wrapper.xml`).

**Known limitation for a future pick planner (Phase 3+):** the wrist's palm
geometry (`gripper_base`, the largest collision mesh on `link7`) extends
~16 cm along the approach direction. Driving straight at the object
horizontally lets the palm reach the object before the fingertips close on
it, pushing the cube instead of grasping it — a working pick needs a
steeper/top-down wrist angle (`joint6`) or a palm-clear approach path. This
scene provides the object and its pose feedback; grasp trajectory planning is
out of scope here.

**Cosmetic-only rendering note:** with `--viewer` (or any offscreen render),
the floor's shadowed regions show light dithering speckle — MuJoCo's
shadow-edge anti-banding noise. It's confined to shadow areas, does not
affect physics or any published topic, and was left as-is.

## Architecture

The **sim layer** owns the physics and the ROS boundary, not the primitive:

- `sim/native/` — MuJoCo runtime + vendored Nero controller (`NeroRuntime`).
- `sim/bridge/bridge_node.py` — ROS 2 bridge. Publishes `/joint_states`
  (joint1..joint7 + `gripper` width), `/feedback/tcp_pose`, `/clock`, `/tf`;
  subscribes `/joint_command`; serves `/move_home`, `/emergency_stop`,
  `/control_enable` (std_srvs); exposes MoveIt `FollowJointTrajectory` actions
  on `/arm_controller/follow_joint_trajectory` and
  `/gripper_controller/follow_joint_trajectory`.

The **primitive** is a thin Robonix boot-layer wrapper:

- `on_activate` spawns `sim/start.sh`, waits for the first `/joint_states`
  (sentinel), then declares `joint_states` / `joint_command` / `end_pose` on
  Atlas. `arm/driver` is auto-declared by the framework.
- MCP tools `status` / `set_enabled` / `emergency_stop` / `move_home` /
  `follow_joint_trajectory` call the bridge's ROS services / actions through
  the shared `RosBackend` node.

## Prerequisites

- MuJoCo 3.3.x + rclpy Python 3.10 venv (used by `sim/start.sh`); ships as the
  repo-local `.venv` — recreate it anytime with `bash scripts/setup_venv.sh`, or
  point `NERO_PYTHON` / `NERO_VENV` at another venv.
- ROS 2 Humble (`/opt/ros/humble`).
- `rbnx` CLI + a registered Robonix source tree (`rbnx setup`).
- `uv` (creates the primitive venv during build).

## Build

```bash
cd primitives/nero_arm
bash scripts/build.sh          # venv + rbnx codegen --mcp -> rbnx-build/codegen/
```

## Run

Bring up the whole stack (system services + the primitive):

```bash
source .env                       # loads VLM_BASE_URL/API_KEY/MODEL (gitignored)
rbnx boot -f robonix_manifest.yaml
```

The manifest omits `pilot` and `liaison` (Phase 3+ VLM planning / chat UI).
`pilot`'s VLM credentials are not stored in the manifest — its `vlm.upstream` /
`api_key` / `model` are `${VLM_BASE_URL}` / `${VLM_API_KEY}` / `${VLM_MODEL}`
placeholders that `rbnx boot` expands from the process environment, so source
`.env` first (gitignored; see the comment in `robonix_manifest.yaml`). The
Phase 1+2 stack is atlas + executor + soma + the primitive.

The primitive spawns the sim on activation. To run the sim layer by itself
(for debugging the bridge):

```bash
bash sim/start.sh              # or --viewer for a mujoco.viewer window
```

### Teardown note

After `rbnx boot` is interrupted (Ctrl-C) or `rbnx shutdown` runs, the
primitive's python process may be left orphaned. This is a **framework-binary
issue, not a package issue**: the installed `rbnx`/`robonix-soma` binaries
predate the `RBNX_DEPLOY_MANAGED` process-group fix (commit `1fce8877`), so
soma records the `rbnx start` wrapper's pgid while the real python runs in its
own group and survives the wrapper's teardown. The primitive itself handles
`CMD_SHUTDOWN` (kills the sim, shuts down rclpy) and `SIGTERM` (clean exit)
correctly. Rebuild the robonix binaries from a current checkout to fix the
orphaning, or clean up manually:

```bash
pkill -f nero_arm.main
```

## Control (joint space)

```bash
# named joint target (radians for joint1..7; gripper opening in metres)
ros2 topic pub /joint_command sensor_msgs/msg/JointState \
  "{name: [joint1,joint2,joint3,joint4,joint5,joint6,joint7,gripper], \
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]}"
```

Note: the sim retunes the vendored arm/gripper servos on load — see
"Servo retune" below — so a commanded pose settles in a few seconds rather
than the vendored gains' tens-to-hundreds of seconds. `status` /
`emergency_stop` / `move_home` are available as MCP tools and as bridge ROS
services (`/control_enable`, `/emergency_stop`, `/move_home`).

### Servo retune

The vendored gains (`damping=2000`, `kp=10..80` per joint — see
`vendor/nero_arm.xml`) make each joint's PD position servo so overdamped that
a commanded move takes tens to hundreds of seconds to arrive, *and* a plain
PD servo has no integral term, so gravity torque leaves a permanent
steady-state error of roughly `gravity_torque / kp` regardless of damping —
verified on this model: the vendored gains settle to the *same* ~20%-off
target that a retuned servo reaches in seconds, just after several minutes
instead. `sim/native/runtime.py` retunes on load (via `MjSpec`, without
editing the vendor file): arm joints get `damping=100` and `kp`/`kv` scaled
5x; the gripper (whose actuator `forcerange` is 10x smaller than the arm's)
gets its own `damping=20, kp=400, kv=25`. Result: a representative
`move_home` bend (`joint2=0.4, joint4=0.2`) settles within <5% of target in
2-5s instead of drifting toward a 20%-off point over minutes. To use the
untuned vendored values, load `assets/robots/nero_arm/vendor/nero_arm.xml`
directly instead of going through `NeroRuntime`.

## Control (trajectory / MoveIt)

The bridge also exposes the standard MoveIt execution interface, so
`move_group` can plan in joint or Cartesian space and drive the sim the same
way it drives a ros2_control controller:

- `/arm_controller/follow_joint_trajectory` — joints `joint1..joint7`.
- `/gripper_controller/follow_joint_trajectory` — joints
  `gripper_joint1`/`gripper_joint2`, folded into the single finger width.

These match the MoveIt config's `moveit_controllers.yaml` controller names.
The executor paces each trajectory point against wall clock and reports success
once the final point is commanded; the arm may still be physically settling
(a few seconds, see "Servo retune" above) after the goal returns — the true
pose stays visible on `/joint_states`, so MoveIt plans subsequent goals from
the real state.

## MoveIt planning / Cartesian motion

MoveIt planning is done by an external `move_group` node (not by the primitive
itself — the primitive only *executes* pre-computed joint trajectories). The
MoveIt config is self-contained in `moveit/nero_gripper_moveit_config` (robot
name `nero`, URDF copied from `urdf/nero_arm.urdf`, no `agx_arm_description`
dependency), and `moveit/cartesian_demo.py` drives the standard Cartesian
pipeline headlessly:

1. `bash sim/start.sh` — the MuJoCo bridge (publishes `/joint_states` + `/tf`,
   serves the `FollowJointTrajectory` actions).
2. `ros2 launch nero_gripper_moveit_config move_group.launch.py` — MoveIt
   planner (exposes `/compute_fk`, `/compute_ik`, `/compute_cartesian_path`).
3. `python3 moveit/cartesian_demo.py 0.05 0 0` — FK the flange → Cartesian path
   → execute via `/arm_controller/follow_joint_trajectory`.

See `moveit/README.md` for build (colcon overlay) and run details.

Verified headlessly: move_group loads the `nero` model, FK/IK/Cartesian all
succeed, and a 5 cm straight-line flange move executes on the sim (fraction
1.0). Two notes: the arm's home pose is straight (near-singular), so the demo
first bends the elbow via the same action before planning; and `/joint_states`
carries a single `gripper` width rather than `gripper_joint1/2`, so move_group
logs "missing gripper_joint1, gripper_joint2" — arm planning is unaffected, but
the gripper group's state is stale (documented limitation).

## Scope

Included: sim + joint-space arm control, gripper, status/enable/estop/home,
the tabletop pick scene (floor/table/grasp-target object) with object-pose
feedback (`/feedback/object_pose`), MoveIt `FollowJointTrajectory` execution
interface (bridge actions + primitive `follow_joint_trajectory` MCP tool), and a
self-contained MoveIt planning config + headless Cartesian demo (`moveit/`).
Not included (Phase 3+): cameras, autonomous pick planning, VLM planning
(`pilot`/`liaison`).
