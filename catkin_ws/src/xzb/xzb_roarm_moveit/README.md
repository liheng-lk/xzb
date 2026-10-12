# xzb_roarm_moveit

ROS1 Noetic MoveIt1 preparation for RoArm-M2-S. The `hand` group is the 3-DOF `base_link -> hand_tcp` positioning chain and `gripper` is separate.

KDL is the initial Noetic solver. The official ROS2 IKFast translation3d solver remains a reference; the standalone `xzb_roarm_control` FK has already been checked against an official IKFast FK reference point.

Trajectory execution is **disabled by default**. Enable the optional `roarm_trajectory_server.py` only after the bounded joint tests pass on real hardware. MoveIt is not the safety authority: every waypoint still goes through `/roarm/move_joints` and its 0.05-rad step guard.
