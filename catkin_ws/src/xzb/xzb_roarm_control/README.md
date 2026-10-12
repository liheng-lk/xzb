# xzb_roarm_control

ROS1 Noetic safety-gated control and 3-DOF position kinematics for RoArm-M2-S.

The node starts with **motion disabled**. It will only enable after a fresh connected state exists. Every command is limited to 0.05 rad per joint and Cartesian delta commands are limited to 0.01 m per axis / 0.015 m norm by default.

The standard `/joint_states` use the official model convention. `/roarm/tool_pose` is FK-derived in `base_link`. The raw controller XYZ remains diagnostic because its origin differs from the URDF `base_link` frame.

`/roarm/soft_stop` disables the command gate. It is not an electrical emergency stop or servo power cut.

The protocol `spd` field is intentionally kept at the official ROS-driver value 0 until its physical scale is characterized; `speed_scale` is validated but does not yet alter firmware speed.

For first hardware testing use `arm_test_cli.py joint-delta` with 0.03 rad increments. Do not start with full gripper or large absolute moves.
