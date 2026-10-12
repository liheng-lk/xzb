# xzb_roarm_driver

ROS1 Noetic serial/protocol driver for Waveshare RoArm-M2-S.

## Safety defaults

The driver owns the USB serial port exclusively and never exposes a generic raw-JSON ROS command. It never deliberately toggles DTR/RTS. Real-hardware testing showed DTR/RTS changes can reset the controller and trigger initialization movement.

`roarm_driver.launch` starts with `command_enable:=false`, so its default behavior remains Phase-1 read-only telemetry using only `T=105` state queries. The internal `T=102` service is usable only when the full `xzb_roarm_control` stack starts the driver with `command_enable:=true`.

Even with driver commands enabled, callers should not call `/roarm/driver/joint_command` directly. Normal motion goes through `xzb_roarm_control`, which applies connection, freshness, hard-limit and per-step checks.

## Serial

- development device: `/dev/ttyUSB0`
- CP2102N, VID:PID `10c4:ea60`
- USB serial `94d2396dbf6eef118409b57dc169b110`
- UART `115200 8N1`

## State

- `/roarm/connected`
- `/roarm/controller_joint_states`: controller-native `b,s,e,t`
- `/roarm/position`: controller XYZ converted mm -> m
- `/roarm/raw_response`
- `/roarm/pose`: deprecated compatibility topic; its identity quaternion is not measured orientation

Controller XYZ and URDF FK use different origins on the currently observed hardware. Do not compare their absolute values without a verified frame transform; use `/roarm/tool_pose` from `xzb_roarm_control` for model-space `base_link -> hand_tcp`.
