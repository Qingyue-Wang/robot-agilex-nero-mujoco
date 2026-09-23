# Upstream: Agilex Nero MuJoCo model

This body package wraps the Agilex Nero arm MuJoCo model that ships with the
`tutorial_for_mujoco` repository (`model/agilex_nero/`).

- **Upstream location:** `tutorial_for_mujoco/model/agilex_nero/nero_arm.xml`
- **Vendored into:** `vendor/nero_arm.xml` (kept byte-identical to upstream)
- **Meshes:** `vendor/assets/*.obj`, `vendor/assets/*.stl`

The upstream model is the fixed-base variant of the arm (bodies `base_link`,
`link1`..`link7`, `gripper_link1`/`gripper_link2`), with 7 revolute joints
(`joint1`..`joint7`) and a 2-finger parallel gripper (`gripper_joint1`/
`gripper_joint2` mirrored by an `<equality>` constraint, driven by the single
`gripper` actuator).

Do not edit `vendor/` in place. To change the model, either upgrade the whole
`vendor/` tree from upstream, or add a thin patch in `robot_wrapper.xml`.
