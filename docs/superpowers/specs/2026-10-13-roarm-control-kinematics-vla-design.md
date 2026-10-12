# RoArm-M2-S Control, Kinematics and VLA Preparation Design

## Scope

XZB remains Ubuntu 20.04 + ROS1 Noetic + catkin. Waveshare ROS2 Humble packages are reference material only; ROS2 runtime is not introduced.

This stage turns the RoArm-M2-S integration from a read-only telemetry driver into a complete ROS1 arm subsystem with deterministic motion control, FK/IK support, MoveIt1 preparation, safe VLA interfaces, and the calibration hooks needed for the later D435i eye-in-hand stage.

The stage ends before production VLA motion is enabled. VLA execution remains dry-run until deterministic motion, kinematics, limits, stop behavior and real-hardware tests pass.

## Verified hardware and transport

- Arm: Waveshare RoArm-M2-S
- Primary transport: USB serial
- Port during development: `/dev/ttyUSB0`
- USB-UART: Silicon Labs CP2102N
- VID:PID: `10c4:ea60`
- USB serial: `94d2396dbf6eef118409b57dc169b110`
- UART: `115200 8N1`
- One serial owner only
- Never toggle DTR/RTS in normal runtime because real hardware testing showed this can reset the controller and trigger initialization movement

A stable `/dev/roarm` udev alias is desirable but not required for the first control validation.

## Source-of-truth references

The implementation follows the controller JSON protocol already verified on the real arm and uses the Waveshare `ros2-humble` source only to verify command semantics, URDF geometry, SRDF grouping and joint conventions.

Important official observations:

- `T=105` queries state and `T=1051` reports `x,y,z,b,s,e,t,torB,torS,torE,torH`.
- The official ROS driver sends joint motion using `T=102`.
- The official ROS driver maps URDF joints to controller joints as:
  - controller base = `-base_link_to_link1`
  - controller shoulder = `-link1_to_link2`
  - controller elbow = `link2_to_link3`
  - controller hand/gripper = `pi - link3_to_gripper_link`
- The official SRDF planning group `hand` contains only the base, shoulder and elbow joints and uses the chain `base_link -> hand_tcp`.
- The gripper is a separate group using `link3_to_gripper_link`.
- Therefore the physical arm positioning chain is 3-DOF plus a gripper joint. The gripper value must not be treated as an independent Cartesian wrist orientation DOF.

## Architecture

```text
D435i / future perception
        |
        v
VLA Adapter (dry-run by default)
        |
        v
Arm Skill / Action API
        |
        v
xzb_roarm_control
  - safety gate
  - joint control
  - Cartesian control
  - FK / IK
  - presets
  - gripper
  - trajectory execution
        |
        v
xzb_roarm_driver
  - one serial owner
  - protocol encode/decode
  - state feedback
  - reconnect
        |
        v
RoArm-M2-S controller
```

In parallel:

```text
xzb_description -> robot_state_publisher -> TF -> MoveIt1
                                              |
                                              v
                                     xzb_roarm_control
```

The VLA, Agent and UI never write raw serial data and never bypass `xzb_roarm_control` safety validation.

## Package responsibilities

### `xzb_roarm_driver`

Owns the serial port and protocol only.

Responsibilities:

- maintain connection and reconnect
- parse mixed JSON + firmware text
- publish raw controller telemetry
- encode a strict whitelist of controller commands
- expose bounded driver services/topics to `xzb_roarm_control`
- never expose a generic raw JSON command ROS API

Driver-level command support for this stage:

- state query
- multi-joint target
- Cartesian XYZ target
- gripper target
- stop command where protocol semantics are verified

Initialization, torque-mode and any other command whose behavior has not been physically validated remain disabled by default.

### `xzb_description`

Contains the ROS1 robot model derived from the official RoArm geometry.

Required joints:

- `base_link_to_link1`
- `link1_to_link2`
- `link2_to_link3`
- `link3_to_gripper_link`

Required TCP link:

- `hand_tcp`

The initial model preserves official geometry and limits. Any XZB mounting transform, D435i transform and calibrated TCP correction are layered outside the vendor geometry so calibration values can change without rewriting the arm model.

### `xzb_roarm_control`

Owns safe arm behavior.

It converts between controller-space and ROS joint-space, applies limits, computes/requests kinematics, executes relative moves, manages named poses and provides action/service interfaces for human tools, MoveIt1 and VLA.

No caller may request an unbounded joint target or unbounded Cartesian step.

### `xzb_interfaces`

Defines the small set of stable robot-facing interfaces needed by the arm stack. Interfaces should be semantic rather than protocol-specific so later HTTP backup transport or a different solver does not change clients.

## State interfaces

Driver outputs:

- `/roarm/connected` — `std_msgs/Bool`
- `/roarm/raw_response` — `std_msgs/String`
- `/roarm/controller_joint_states` — `sensor_msgs/JointState`, controller-native `b/s/e/t`
- `/roarm/position` — `geometry_msgs/Point`, controller XYZ converted mm -> m

Control outputs:

- `/joint_states` — ROS-model joint values using official sign mapping
- `/roarm/tool_pose` — FK-derived `geometry_msgs/PoseStamped` for `hand_tcp`
- `/roarm/control_state` — status string/state message for idle, moving, stopped, rejected and fault conditions

The previous Phase 1 `/roarm/pose` identity-quaternion placeholder is deprecated. A measured XYZ position must not be presented as a measured 6D pose.

## Joint conventions

Controller feedback:

- `b`, `s`, `e`, `t` are controller radians

ROS model values:

- `base_link_to_link1 = -b`
- `link1_to_link2 = -s`
- `link2_to_link3 = e`
- `link3_to_gripper_link = pi - t`

The inverse mapping is used before sending `T=102`.

Velocity and effort arrays remain empty until trustworthy units are available. `torB/torS/torE/torH` remain raw diagnostic values.

## Safety model

All command paths pass through one validator.

Initial software position limits follow the official model:

- base: `[-3.1416, 3.1416]`
- shoulder: `[-1.5708, 1.5708]`
- elbow: `[-1.0, 3.1416]`
- gripper model joint: `[0.0, 1.5]`

These are hard software bounds, not permission to explore mechanical extremes during first hardware tests.

Additional first-test limits:

- joint relative step default max: `0.05 rad`
- Cartesian relative step default max per axis: `0.01 m`
- Cartesian norm default max: `0.015 m`
- first-test speed scaling: conservative
- commands are rejected if state is stale or disconnected
- absolute presets are disabled until their values are explicitly validated on hardware

The later whole-robot collision layer will add the rear-base exclusion zone required by the mast/linear actuator. It is not guessed in this stage.

## Motion interfaces

### Joint control

Support both absolute and relative joint requests.

- absolute joint requests are primarily for deterministic tools and MoveIt1
- relative requests are the default hardware-validation method
- the control layer converts ROS-model joints to controller-space before calling the driver

### Gripper

Gripper is controlled independently from the 3-DOF positioning chain.

Provide semantic commands:

- set normalized opening `[0,1]`
- set model joint radians
- open
- close

Open/close endpoint values are configuration parameters so real-hardware testing can refine them.

### Cartesian control

Use controller Cartesian command support for low-level execution only after bounded validation.

High-level Cartesian requests are expressed in the arm base frame in metres. The control layer validates workspace and delta size before converting metres to controller millimetres.

VLA does not get access to absolute Cartesian teleport commands. Its default action mode is bounded Cartesian delta.

### Stop

Provide a single semantic `/roarm/stop` operation. If a firmware-level motion stop command is available and verified, the driver uses it. Otherwise control stops issuing new targets and reports that only command-stream stop, not servo power cut, is available.

The system must never label a soft command stop as an electrical emergency stop.

## Forward and inverse kinematics

### Forward kinematics

The authoritative software FK chain is `base_link -> hand_tcp` from the ROS1 URDF. FK output is used for:

- `/roarm/tool_pose`
- MoveIt1 consistency checks
- hand-eye calibration preparation
- VLA observation state

### Inverse kinematics

The first ROS1 solver is MoveIt1 KDL for reliability and maintainability on Noetic. The official ROS2 IKFast setup is retained as a reference and can be ported later if solver speed or determinism requires it.

The `hand` planning group contains 3 positioning joints. IK requests therefore target reachable TCP position and only compatible orientation constraints; arbitrary 6D orientation goals are not promised by a 3-DOF positioning chain.

For VLA and first grasping work, position or small Cartesian-delta targets are preferred over arbitrary quaternion targets.

## MoveIt1 preparation

Create a ROS1 MoveIt configuration for planning group `hand` and gripper group `gripper`.

Requirements:

- ROS1 Noetic MoveIt1 only
- base link `base_link`
- tip link `hand_tcp`
- KDL solver initially
- conservative scaling defaults
- fake execution mode for offline validation
- a real execution bridge that sends validated trajectory waypoints through `xzb_roarm_control`, not directly to serial

MoveIt1 is a planner and kinematics client, not the safety authority.

## VLA preparation — mode C

Both action representations exist, but production VLA defaults to Cartesian delta.

### Mode A: joint action

Debug/engineering action vector:

```text
[q_base, q_shoulder, q_elbow, q_gripper]
```

This mode is available only through the safety gateway and is disabled for normal VLA inference by default.

### Mode B: Cartesian delta + gripper

Default VLA action vector:

```text
[dx, dy, dz, gripper]
```

Units:

- `dx,dy,dz`: metres in the declared arm-base or tool-relative frame
- `gripper`: normalized `[0,1]`

There is no fake independent `dtool` orientation channel because the official model shows only 3 positioning joints plus a separate gripper joint.

### Observations

The adapter prepares synchronized observations containing at minimum:

- D435i RGB frame hook
- D435i depth frame hook
- ROS-model joint state
- FK TCP pose
- gripper state
- arm connected/fault state
- timestamp

Camera hooks may be inactive until D435i hand-eye calibration is completed.

### Dry-run behavior

`dry_run=true` is the default and mandatory before hardware acceptance.

In dry-run:

- model/action inputs are accepted
- normalization and safety validation run
- the resulting semantic arm action is published/logged
- no motion command reaches `xzb_roarm_driver`

Real VLA execution requires an explicit runtime enable plus successful arm-health state.

### Dataset preparation

Provide a recorder-friendly observation/action schema compatible with later LeRobot/SmolVLA conversion. The ROS stack stores semantic observations/actions and timestamps; model-specific tensor preprocessing stays in the VLA adapter layer.

## Hand-eye and grasping boundary for the next stage

After deterministic arm motion and FK/IK pass hardware tests, the next stage adds:

- D435i intrinsics verification
- camera optical frame definition
- eye-in-hand hand-eye calibration
- `hand_tcp -> camera_link` transform
- calibrated TCP correction/offset
- camera/depth target -> arm base coordinate conversion
- grasp approach/pre-grasp/grasp/retreat skills

The current stage must therefore keep camera extrinsics and TCP offsets parameterized rather than hard-coded.

## Test strategy

### Offline tests

- protocol command encoding contains only whitelisted T codes
- controller <-> ROS joint mapping round-trips
- hard and relative-step limits reject unsafe requests
- Cartesian unit conversion is correct
- disconnected/stale state rejects motion
- VLA dry-run cannot reach the driver
- Mode B clamps/rejects oversized deltas
- FK/MoveIt configuration loads with the expected joint names and tip link

### Real-hardware progression

1. Phase 1 read-only state validation
2. 0.03-0.05 rad base relative move
3. return to recorded starting angle
4. repeat one joint at a time
5. gripper small/open-close validation
6. multi-joint small target
7. 5 mm Cartesian move on one axis
8. compare controller XYZ and FK trend
9. stop behavior
10. USB reconnect with motion disabled after reconnect until fresh state exists
11. only then enable wider deterministic motion
12. VLA remains dry-run until all above pass

## Acceptance

This stage is ready for user hardware acceptance when:

1. all ROS1/catkin packages build on Noetic
2. unit tests pass
3. driver keeps a single serial owner and never toggles DTR/RTS
4. ROS/model joint mapping is explicit and tested
5. bounded joint, Cartesian and gripper interfaces exist
6. FK publishes `hand_tcp` pose from the robot model
7. IK/MoveIt1 configuration loads and solves representative reachable targets offline
8. safety validator blocks invalid, stale and oversized commands
9. VLA Mode A and Mode B adapters exist with dry-run default
10. no VLA path can bypass the control/safety layer
11. hand-eye calibration inputs and TCP-offset parameters have defined integration points
12. a step-by-step real-hardware test document is included
