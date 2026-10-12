# Validation performed for this replacement

## Replacement boundary

- Only `linear_actuator_control/` is modified or extended.
- All **89** original files outside the package were compared byte-for-byte.
- No file outside that package was added, removed, or modified.
- `firmware_reference/main.c` equals the supplied v2 `Src/main.c` byte-for-byte.
- Firmware SHA-256: `ef87b1ed7f79e8da7184d550b8f7eb913389f6a38fd0d615075f7c6f3bd30738`

## Executed checks

- 30 actuator/servo bridge regression tests: PASS. These use a simulated clock,
  mocked rospy and mocked serial transport; no hardware is driven.
- 5 existing platform runtime-configuration regression tests: PASS.
- Python AST/syntax parsing for the package's four Python files: PASS.
- XML parsing for package.xml and the launch file: PASS.
- Existing launch argument names used by robotctl: preserved and checked.
- `gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only` on the supplied main.c: PASS.
  This is a host-side syntax check, not an ARM firmware build.
- Final archives: ZIP CRC/integrity and replacement-boundary checks performed
  by the packaging script before delivery.

## Not performed

No ROS1 Noetic/catkin build, live ROS graph test, ARM linking, STM32 flashing,
serial electrical verification, or physical actuator/servo test was possible
in this environment. Mechanical limits, actual device port, both servo baud
settings, and movement must be verified on the robot before normal operation.
No servo emergency-stop capability is claimed; the supplied S command brakes
only the linear actuator.
