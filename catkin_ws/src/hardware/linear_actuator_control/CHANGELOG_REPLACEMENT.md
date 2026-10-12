# Replacement scope: actuator + two ST3215 servos

## Inputs

- Platform: `xunjian-robot-platform-main.zip`
- Protocol/firmware: `STM32_ST3215_dual_servo_v2(1).zip`
- Replacement boundary: `linear_actuator_control/` only.

All pre-existing platform files outside this directory are retained byte-for-byte.
In particular, camera_mast_manager, chassis/navigation, arm packages, root config,
robotctl, backend, frontend and existing CI definitions are not modified.

## Preserved interfaces

- ROS package: `linear_actuator_control`
- Node/script: `linear_actuator_node` / `linear_actuator_node.py`
- Launch: `linear_actuator.launch`
- Commands/status: `/linear_actuator/cmd`, `/linear_actuator/status`
- Original actuator aliases and `actuator_keyboard.py`
- Port default `/dev/linear_actuator`, PC UART 115200, original udev script
- Existing launch arguments used by `scripts/robotctl.py`

## Replacement details

1. Reference main.c replaced with the exact supplied v2 source; original upstream
   README retained separately. No changes to the supplied STM32 firmware.
2. ROS bridge adds PAN/TILT/PT/PING/READ/BAUD/HELP, validated integer angles,
   configurable per-axis limits, and `/linear_actuator/servo_cmd`.
3. Newline framing and serialized/paced transmission replace the old raw-byte
   status polling. The firmware's missing '?' command is never written to UART.
4. Startup verifies v2 HELP and UART2 baud acknowledgement before accepting
   motion; servos are not automatically centered/homed.
5. Actuator refreshes continue alongside servo transactions. Host timeout is
   not reset by repeated movement requests or direction changes. Disconnect,
   watchdog stop and MCU reboot clear pending motion and servo requests.
6. Added interactive `actuator_servo_console.py`, declared python3-serial runtime
   dependency, updated package documentation and launch parameters.
7. Added offline mock-based regression tests inside the same package.

## Deliberate limitations

No absolute mast height or automatic coordinated mission sequence is invented.
No web UI servo controls are added. S and shutdown stop only the actuator:
firmware has no servo emergency-stop command. SENT does not confirm movement.
The source archive is not a full STM32 build project. Hardware and a ROS1 Noetic
runtime are not available in this editing environment; tests do not certify
real-machine behavior or mechanical clearance.
