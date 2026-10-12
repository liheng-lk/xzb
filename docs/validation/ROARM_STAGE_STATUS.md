# RoArm stage status

Software prepared on branch `feat/roarm-control-kinematics-vla`:

- single-owner serial state driver
- strict T=102 joint encoder behind disabled-by-default internal command gate
- official controller/ROS joint sign mapping
- software hard limits and per-step guards
- standalone 3-DOF FK and numerical position IK
- FK checked offline against the official generated IKFast solver reference formula
- ROS1 description with `hand_tcp` and uncalibrated TCP/D435i hooks
- semantic joint, Cartesian, gripper, FK and IK services
- explicit motion-enable and software-stop gate
- MoveIt1/KDL planning configuration with trajectory execution disabled by default
- optional FollowJointTrajectory bridge behind the same safe control service
- VLA mode C (joint engineering mode + default Cartesian delta mode)
- VLA dry-run/hardware-enable dual gate
- semantic dataset recorder hooks for `/arm_camera/*`

Not claimed as validated until the user runs on the Jetson Orin Nano:

- catkin/MoveIt1 runtime build
- physical joint directions and limits
- gripper endpoint semantics
- real Cartesian move accuracy
- trajectory execution timing
- controller-XYZ to URDF-base transform
- hand-eye calibration
- TCP/camera offsets
- physical grasping
- VLA hardware execution
