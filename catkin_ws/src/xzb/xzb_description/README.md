# xzb_description

ROS1 Noetic model of RoArm-M2-S. Joint origins, axes, limits and `hand_tcp` follow the official Waveshare model. Primitive visual/collision geometry is intentionally conservative and does not claim STL-level collision fidelity.

`tcp_calibrated` and `arm_camera_mount_link` are zero-offset hooks until the later eye-in-hand/TCP calibration stage. A zero value means **uncalibrated**, not measured.
