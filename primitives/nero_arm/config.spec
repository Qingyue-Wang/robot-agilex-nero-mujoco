config:
  joint_states_topic:
    type: string
    default: /joint_states
    description: Absolute measured joint feedback topic (joint1..joint7 + gripper width).
    example: /joint_states
    failure: CMD_ACTIVATE fails if no JointState arrives on this topic before sentinel_timeout_s.
  joint_command_topic:
    type: string
    default: /joint_command
    description: Absolute named JointState command topic for arm joints and gripper.
    example: /joint_command
    failure: CMD_ACTIVATE declares this topic; commands are ignored while the control gate is closed.
  end_pose_topic:
    type: string
    default: /feedback/tcp_pose
    description: Measured end-effector pose topic (geometry_msgs/Pose, frame base_link).
    example: /feedback/tcp_pose
    failure: CMD_ACTIVATE declares this topic; no pose is published until the sim is up.
  arm_controller_action:
    type: string
    default: /arm_controller/follow_joint_trajectory
    description: FollowJointTrajectory action for joint1..joint7 (MoveIt arm_controller).
    example: /arm_controller/follow_joint_trajectory
    failure: follow_joint_trajectory fails while this action server is unavailable.
  gripper_controller_action:
    type: string
    default: /gripper_controller/follow_joint_trajectory
    description: FollowJointTrajectory action for the two mirrored gripper joints.
    example: /gripper_controller/follow_joint_trajectory
    failure: follow_joint_trajectory fails while this action server is unavailable.
  sentinel_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: Maximum wait for the first joint feedback after spawning the sim.
    example: 90
    failure: CMD_INIT fails on a non-numeric, non-positive value; CMD_ACTIVATE fails when it expires.
  viewer:
    type: bool
    default: false
    description: Open a mujoco.viewer window (requires a display).
    example: false
    failure: Passed through to sim/start.sh; no display means the sim still runs headless.
  sim_python:
    type: string
    default: ""
    description: Interpreter override for the spawned sim (passed as NERO_PYTHON).
    example: /home/wqy/tutorial_for_mujoco/.venv_ros2/bin/python
    failure: If empty, sim/start.sh auto-detects a Python 3.10 venv with MuJoCo + rclpy.
