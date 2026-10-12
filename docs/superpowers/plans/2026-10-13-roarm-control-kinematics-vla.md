# RoArm Control, Kinematics and VLA Preparation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a ROS1 Noetic RoArm-M2-S arm subsystem with bounded joint/gripper/Cartesian control, standalone FK/IK, MoveIt1 preparation, and a dry-run-first VLA action gateway.

**Architecture:** Keep serial/protocol ownership in `xzb_roarm_driver`, safety and kinematics in `xzb_roarm_control`, the robot model in `xzb_description`, public service/message contracts in `xzb_interfaces`, MoveIt1 configuration in `xzb_roarm_moveit`, and VLA normalization/dry-run gating in `xzb_runtime_bridge`. All motion reaches the controller through validated semantic commands; no raw serial/JSON interface is exposed to Agent/VLA code.

**Tech Stack:** Ubuntu 20.04, ROS1 Noetic, catkin, Python 3, rospy, NumPy, sensor_msgs, geometry_msgs, trajectory_msgs, control_msgs, actionlib, robot_state_publisher, MoveIt1/KDL.

**Spec:** `docs/superpowers/specs/2026-10-13-roarm-control-kinematics-vla-design.md`

## Global Constraints

- ROS1 Noetic only; Waveshare ROS2 Humble code is reference only.
- One serial owner; no runtime DTR/RTS toggling.
- Hardware control remains disabled by default for VLA (`dry_run=true`, `hardware_enable=false`).
- First hardware motions are relative and bounded to 0.05 rad joint steps or 0.01 m per Cartesian axis.
- VLA default action is `[dx,dy,dz,gripper]`; joint-mode action remains engineering-only by default.
- `hand_tcp` is controlled by three positioning joints; gripper is not an independent Cartesian orientation DOF.

## Review Focus

- Controller/ROS sign conversion must round-trip exactly.
- Stale or disconnected state must reject every motion path.
- Invalid NaN/Inf, out-of-limit joint requests and oversized Cartesian deltas must be rejected.
- IK must fail cleanly for unreachable points rather than returning unsafe boundary values.
- VLA dry-run must never invoke the hardware dispatch callback.

---

### Task 1: Extend protocol command encoding

**Files:**
- Modify: `catkin_ws/src/xzb/xzb_roarm_driver/src/xzb_roarm_driver/protocol.py`
- Test: `catkin_ws/src/xzb/xzb_roarm_driver/test/test_protocol.py`

**Interfaces:**
- Produces: `make_joint_command(base, shoulder, elbow, hand, speed, acceleration) -> bytes`

- [ ] Write tests for exact `T=102` JSON fields, finite numeric validation and newline framing.
- [ ] Verify tests fail before implementation.
- [ ] Implement only whitelisted `T=102` command encoding.
- [ ] Run the driver pure-Python test suite.

### Task 2: Add controller/ROS mapping and safety core

**Files:**
- Create: `catkin_ws/src/xzb/xzb_roarm_control/src/xzb_roarm_control/joint_mapping.py`
- Create: `catkin_ws/src/xzb/xzb_roarm_control/src/xzb_roarm_control/safety.py`
- Test: `catkin_ws/src/xzb/xzb_roarm_control/test/test_joint_mapping.py`
- Test: `catkin_ws/src/xzb/xzb_roarm_control/test/test_safety.py`

**Interfaces:**
- Produces: `controller_to_ros(values)`, `ros_to_controller(values)`, `ArmSafety.validate_joint_target`, `ArmSafety.validate_joint_delta`, `ArmSafety.validate_cartesian_delta`.

- [ ] Test official sign mapping and round-trip behavior.
- [ ] Test hard limits, relative-step limits, NaN/Inf and Cartesian delta limits.
- [ ] Implement mapping and validation.
- [ ] Run pure-Python control tests.

### Task 3: Implement standalone FK/IK model

**Files:**
- Create: `catkin_ws/src/xzb/xzb_roarm_control/src/xzb_roarm_control/kinematics.py`
- Test: `catkin_ws/src/xzb/xzb_roarm_control/test/test_kinematics.py`

**Interfaces:**
- Produces: `RoArmKinematics.forward(joints3) -> FKResult`, `RoArmKinematics.inverse(target_xyz, seed) -> IKResult`.

- [ ] Test deterministic FK, finite outputs and FK->IK position round-trip.
- [ ] Test unreachable targets return `success=False` with residual.
- [ ] Implement homogeneous-transform FK from official URDF joint origins/axes.
- [ ] Implement damped-least-squares position IK with joint-limit clamping and bounded iterations.
- [ ] Run pure-Python control tests.

### Task 4: Define ROS arm interfaces

**Files:**
- Create: `catkin_ws/src/xzb/xzb_interfaces/msg/ArmAction.msg`
- Create: `catkin_ws/src/xzb/xzb_interfaces/msg/ArmStatus.msg`
- Create: `catkin_ws/src/xzb/xzb_interfaces/srv/DriverJointCommand.srv`
- Create: `catkin_ws/src/xzb/xzb_interfaces/srv/MoveArmJoints.srv`
- Create: `catkin_ws/src/xzb/xzb_interfaces/srv/MoveArmCartesian.srv`
- Create: `catkin_ws/src/xzb/xzb_interfaces/srv/SetArmGripper.srv`
- Create: `catkin_ws/src/xzb/xzb_interfaces/srv/ComputeArmFK.srv`
- Create: `catkin_ws/src/xzb/xzb_interfaces/srv/SolveArmIK.srv`
- Modify: `catkin_ws/src/xzb/xzb_interfaces/CMakeLists.txt`
- Modify: `catkin_ws/src/xzb/xzb_interfaces/package.xml`

**Interfaces:**
- Produces generated ROS messages/services used by driver, control and VLA adapter.

- [ ] Add message/service declarations and `message_runtime`.
- [ ] Keep interfaces semantic; no raw JSON field.

### Task 5: Upgrade driver ROS node for bounded joint dispatch

**Files:**
- Modify: `catkin_ws/src/xzb/xzb_roarm_driver/src/xzb_roarm_driver/driver_core.py`
- Modify: `catkin_ws/src/xzb/xzb_roarm_driver/src/xzb_roarm_driver/ros_node.py`
- Modify: `catkin_ws/src/xzb/xzb_roarm_driver/package.xml`
- Modify: `catkin_ws/src/xzb/xzb_roarm_driver/CMakeLists.txt`
- Test: `catkin_ws/src/xzb/xzb_roarm_driver/test/test_driver_core.py`

**Interfaces:**
- Consumes: `DriverJointCommand.srv`
- Produces: `/roarm/driver/joint_command`, `/roarm/controller_joint_states`, `/roarm/position`.

- [ ] Test disconnected dispatch rejection and exact command bytes.
- [ ] Implement driver service with finite-value validation and no generic command path.
- [ ] Preserve read-only query/reconnect behavior.

### Task 6: Build ROS1 description and TF-ready model

**Files:**
- Create: `catkin_ws/src/xzb/xzb_description/urdf/roarm_m2s.urdf.xacro`
- Create: `catkin_ws/src/xzb/xzb_description/launch/roarm_description.launch`
- Create: `catkin_ws/src/xzb/xzb_description/config/calibration_offsets.yaml`
- Replace skeleton metadata in `catkin_ws/src/xzb/xzb_description/CMakeLists.txt` and `package.xml` as needed.

**Interfaces:**
- Produces: `robot_description`, `base_link -> hand_tcp`, parameterized future TCP/camera calibration frames.

- [ ] Encode official joint origins, axes and limits.
- [ ] Add `tcp_calibrated` and `arm_camera_mount_link` zero-offset hooks clearly marked uncalibrated.
- [ ] Launch robot_state_publisher against `/joint_states`.

### Task 7: Implement safe ROS control node and hardware test CLI

**Files:**
- Create: `catkin_ws/src/xzb/xzb_roarm_control/setup.py`
- Create: `catkin_ws/src/xzb/xzb_roarm_control/src/xzb_roarm_control/__init__.py`
- Create: `catkin_ws/src/xzb/xzb_roarm_control/src/xzb_roarm_control/control_core.py`
- Create: `catkin_ws/src/xzb/xzb_roarm_control/scripts/roarm_control_node.py`
- Create: `catkin_ws/src/xzb/xzb_roarm_control/scripts/arm_test_cli.py`
- Create: `catkin_ws/src/xzb/xzb_roarm_control/launch/roarm_control.launch`
- Modify: `catkin_ws/src/xzb/xzb_roarm_control/CMakeLists.txt`
- Modify: `catkin_ws/src/xzb/xzb_roarm_control/package.xml`
- Test: `catkin_ws/src/xzb/xzb_roarm_control/test/test_control_core.py`

**Interfaces:**
- Produces: `/roarm/move_joints`, `/roarm/move_cartesian`, `/roarm/set_gripper`, `/roarm/compute_fk`, `/roarm/solve_ik`, `/roarm/tool_pose`, `/roarm/status`.

- [ ] Test stale-state and disabled-motion rejection before hardware dispatch.
- [ ] Test relative joint and Cartesian targets use current state as the seed/reference.
- [ ] Implement service handlers and `hand_tcp` pose publication.
- [ ] Provide CLI commands `status`, `joint-delta`, `cart-delta`, `gripper`, `fk`, `ik`, `disable`, `enable`.

### Task 8: Prepare MoveIt1/KDL package

**Files:**
- Create package: `catkin_ws/src/xzb/xzb_roarm_moveit/`
- Create: `config/roarm.srdf`, `config/kinematics.yaml`, `config/joint_limits.yaml`, `config/ompl_planning.yaml`
- Create: `launch/planning_context.launch`, `launch/move_group.launch`, `launch/demo.launch`

**Interfaces:**
- Produces planning group `hand`, gripper group `gripper`, tip `hand_tcp`, initial KDL solver configuration.

- [ ] Port SRDF grouping semantics from official model.
- [ ] Configure KDL for ROS1 Noetic.
- [ ] Keep planning scaling at 0.1 initially.
- [ ] Do not make MoveIt the safety authority.

### Task 9: Prepare VLA mode-C gateway and dataset schema

**Files:**
- Create: `catkin_ws/src/xzb/xzb_runtime_bridge/setup.py`
- Create: `catkin_ws/src/xzb/xzb_runtime_bridge/src/xzb_runtime_bridge/__init__.py`
- Create: `catkin_ws/src/xzb/xzb_runtime_bridge/src/xzb_runtime_bridge/vla_arm_adapter.py`
- Create: `catkin_ws/src/xzb/xzb_runtime_bridge/src/xzb_runtime_bridge/smolvla_adapter.py`
- Create: `catkin_ws/src/xzb/xzb_runtime_bridge/scripts/vla_arm_adapter_node.py`
- Create: `catkin_ws/src/xzb/xzb_runtime_bridge/scripts/arm_dataset_recorder.py`
- Create: `catkin_ws/src/xzb/xzb_runtime_bridge/docs/VLA_ARM_SCHEMA.md`
- Modify: `catkin_ws/src/xzb/xzb_runtime_bridge/CMakeLists.txt` and `package.xml`
- Test: `catkin_ws/src/xzb/xzb_runtime_bridge/test/test_vla_arm_adapter.py`

**Interfaces:**
- Consumes: `ArmAction.msg`, arm state and optional camera observations.
- Produces: checked semantic arm action, dry-run log/record stream, future predictor adapter interface.

- [ ] Test default dry-run cannot dispatch hardware.
- [ ] Test Cartesian delta and joint-mode validation.
- [ ] Implement callable predictor adapter without a hard LeRobot dependency.
- [ ] Add JSONL observation/action recorder schema with camera-topic hooks.

### Task 10: Documentation and hardware acceptance procedure

**Files:**
- Create: `docs/validation/ROARM_HARDWARE_TEST.md`
- Update: `catkin_ws/src/xzb/xzb_roarm_driver/README.md`
- Create: `catkin_ws/src/xzb/xzb_roarm_control/README.md`

- [ ] Document exact sync/build commands.
- [ ] Document read-only test before motion enable.
- [ ] Document 0.03 rad base test, per-joint return test, gripper test, 5 mm Cartesian test, FK/IK comparison and VLA dry-run.
- [ ] Clearly distinguish soft stop from electrical E-stop.

### Task 11: Verification

- [ ] Run all pure-Python unit tests for driver, control and runtime bridge in a clean environment.
- [ ] Run Python syntax compilation for every new Python file.
- [ ] Inspect branch diff for raw serial/VLA bypass paths.
- [ ] Leave catkin/MoveIt/hardware verification explicitly pending for the user's Orin Nano Noetic machine.
