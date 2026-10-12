# xzb_roarm_driver

ROS1 Noetic read-only state driver for Waveshare RoArm-M2-S.

## Phase 1 safety boundary

Phase 1 is intentionally read-only. The only command this package sends to the arm is the state query:

```json
{"T":105}
```

The driver does **not** send joint, Cartesian, gripper, homing, LED, torque, or other motion/control commands.

Do not add runtime `setDTR()` / `setRTS()` toggles. Real-hardware testing on the CP2102N link showed that changing DTR/RTS can reset the controller and trigger the arm's startup initialization motion.

## Verified serial link

- Device during development: `/dev/ttyUSB0`
- USB-UART: Silicon Labs CP2102N
- VID:PID: `10c4:ea60`
- USB serial: `94d2396dbf6eef118409b57dc169b110`
- UART: `115200 8N1`

A stable `/dev/roarm` udev alias can be added later using the unique USB serial number.

## Protocol

The serial stream may contain all of the following interleaved as newline-delimited records:

- command echo, e.g. `{"T":105}`
- state JSON, `T=1051`
- non-JSON firmware logs, e.g. `Bytes received: 214`

`T=1051` fields used by Phase 1:

- `x/y/z`: controller millimetres, published as metres
- `b/s/e/t`: controller joint values in radians, published unchanged in Phase 1
- `torB/torS/torE/torH`: retained as raw protocol data only; no N-m conversion is assumed

## ROS interfaces

Published topics:

- `/roarm/connected` — `std_msgs/Bool`, latched
- `/roarm/joint_states` — `sensor_msgs/JointState`
- `/roarm/pose` — `geometry_msgs/Pose`
- `/roarm/raw_response` — `std_msgs/String`

Private parameters:

- `~port` default `/dev/ttyUSB0`
- `~baud_rate` default `115200`
- `~query_rate` default `2.0`
- `~reconnect_interval` default `1.0`
- `~serial_timeout` default `0.2`
- `~joint_frame_id` default `roarm_base_link`

### Joint-state convention

Phase 1 publishes the controller values directly using the semantic names:

```text
base, shoulder, elbow, hand
```

No URDF/MoveIt sign conversion is applied yet. The official model's joint names and sign conventions will be integrated and physically verified in the later description/MoveIt1 phase.

### Pose convention

`/roarm/pose.position` is the controller's `x/y/z` converted from millimetres to metres.

The `T=1051` packet does not provide a full end-effector quaternion. Therefore Phase 1 publishes an **identity quaternion only as a position-only placeholder**. Consumers must not interpret the orientation as measured end-effector orientation. Full orientation will be derived from the verified robot model/kinematics in a later phase.

## Launch

```bash
roslaunch xzb_roarm_driver roarm_driver.launch
```

During Phase 1 this should result only in periodic `T=105` state queries.

## Validation order

1. `rosdep install`
2. `catkin build xzb_roarm_driver`
3. run the Python unit tests
4. launch on the real arm and inspect the four ROS topics
5. only after stable state publishing, test USB unplug/replug recovery

Do not send motion commands during this validation phase.
