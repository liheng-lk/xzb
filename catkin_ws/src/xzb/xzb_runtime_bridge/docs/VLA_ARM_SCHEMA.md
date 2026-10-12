# VLA Arm Interface Schema

Default production action mode is Cartesian delta:

`[dx_m, dy_m, dz_m, gripper_0_to_1]`

Engineering-only joint mode is:

`[base_rad, shoulder_rad, elbow_rad, gripper_joint_rad]`

The VLA adapter defaults to `dry_run=true`, `hardware_enable=false`, and `allow_joint_mode=false`. Model output never reaches serial directly; it goes through `xzb_roarm_control` and its fresh-state, joint-step and Cartesian-step guards.

Observation records contain timestamps, ROS-model joint state, FK `hand_tcp` pose, gripper state within the joint vector, arm health/status, checked semantic action, and optional RGB/depth timestamps. Raw image/depth frames should be recorded with rosbag or a dedicated image dataset writer and joined by timestamp during LeRobot/SmolVLA conversion.

The model dependency is intentionally injected: `SmolVLAPredictorAdapter` accepts a callable predictor, so the base ROS package does not require Torch/LeRobot just to start the robot.

Real VLA hardware execution must remain disabled until deterministic arm motion, hand-eye calibration, TCP offset and grasp safety are accepted on the physical robot.
