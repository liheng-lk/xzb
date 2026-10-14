# xzb_roarm_driver

ROS1 Noetic serial/protocol driver for Waveshare RoArm-M2-S.

## Safety defaults

The driver owns the USB serial port exclusively and never exposes a generic raw-JSON ROS command. It never deliberately toggles DTR/RTS. Real-hardware testing showed DTR/RTS changes can reset the controller and trigger initialization movement.

`roarm_driver.launch` starts with `command_enable:=false`, so motion commands remain disabled by default. Read-only operation uses periodic `T=105` telemetry and permits one explicit diagnostic query, `T=602`, which only asks the firmware to print `boot.mission` content.

The internal `T=101`/`T=102` command services are usable only when the full `xzb_roarm_control` stack starts the driver with `command_enable:=true`. Even with driver commands enabled, callers should not call `/roarm/driver/single_joint_command` or `/roarm/driver/joint_command` directly. Normal motion goes through `xzb_roarm_control`, which applies connection, freshness, hard-limit and per-step checks.

No ROS API is provided for arbitrary firmware JSON, PID writes, torque-limit writes, EEPROM writes, calibration writes, homing, or initialization commands.

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
- `/roarm/raw_tx`: audit of bytes successfully written to the controller
- `/roarm/pose`: deprecated compatibility topic; its identity quaternion is not measured orientation

Controller XYZ and URDF FK use different origins on the currently observed hardware. Do not compare their absolute values without a verified frame transform; use `/roarm/tool_pose` from `xzb_roarm_control` for model-space `base_link -> hand_tcp`.

## Read-only boot mission diagnostic

The firmware's `T=602` command prints `boot.mission`. XZB exposes only that exact diagnostic command:

```bash
rosservice call /roarm/driver/query_boot_mission "{}"
rostopic echo -n 1 /roarm/boot_mission
```

`/roarm/boot_mission` is latched and contains the firmware's textual mission dump. Periodic `T=105`/`T=1051` telemetry lines are excluded from this capture so the boot steps remain readable. This diagnostic is available even while `command_enable:=false` because it does not alter motion, PID, torque, calibration, or EEPROM state.
