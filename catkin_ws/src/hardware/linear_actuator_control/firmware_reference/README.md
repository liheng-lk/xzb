# Supplied STM32 ST3215 dual-servo v2 firmware

`main.c` is copied byte-for-byte from `Src/main.c` in the uploaded
`STM32_ST3215_dual_servo_v2(1).zip`. `UPSTREAM_README.txt` preserves that
archive's original notes. The commands below use the actual packaged path
(the upstream notes mention a loose `stm32_dual_servo_main.c` download).

This is a reference source file, NOT a complete STM32 build project and NOT
built by catkin. If the supplied v2 firmware is already running, no additional
firmware change is necessary for this ROS package. Otherwise, use your existing
STM32 project, keep a backup, replace only its `Src/main.c`, then build/flash.

Example after installing this ROS package in `~/catkin_ws/src`:

```bash
cd ~/stm32_ws/linear_actuator_stm32
cp Src/main.c "Src/main.c.bak.$(date +%Y%m%d-%H%M%S)"
cp ~/catkin_ws/src/linear_actuator_control/firmware_reference/main.c Src/main.c
cmake --build build/Debug --clean-first
cube programmer -c port=SWD freq=950 -d build/Debug/linear_actuator_stm32.elf -v -rst
```

The command paths/flash tool above follow the supplied firmware notes; use the
actual location of your STM32 project. No prebuilt ELF/HEX or new CMake/startup
files are included. A failed SWD connection does not flash the firmware.

## Connections (as specified by the supplied source)

- CP2102 TXD -> STM32 PA10 (USART1_RX)
- CP2102 RXD <- STM32 PA9 (USART1_TX)
- CP2102 GND <-> STM32 GND
- STM32 PA2 (USART2_TX) -> Waveshare Bus Servo Adapter (A) **TX**
- STM32 PA3 (USART2_RX) -> Waveshare adapter **RX**
- Both adapter jumpers at **A**, UART mode
- External 12 V -> adapter servo power; common ground with STM32
- Servo ID1 = PAN; ID2 = TILT; use adapter D/V/G sockets
- H bridge unchanged: PB6 -> PWML1; PB7 -> PWML2; PB8 -> EN

Do not power the servos from STM32 3.3 V/5 V or the CP2102 supply.
The adapter TX/RX labels above follow the uploaded firmware's wiring notes.

## Wire protocol

PC link: 115200 baud, 8N1. Servo UART2: default 1000000 baud at reset.

- `U`, `D`, `S`, `F`: immediate actuator commands at an empty line buffer.
- `PAN n`, `TILT n`, `PT p t`: integer positions, 0..360, newline-terminated.
- `PING`, `READ`, `HELP`, `BAUD n`: newline-terminated.
- `BAUD` changes only STM32 UART2; it does not rewrite the servo EEPROM.
- Supported UART2 baud rates: 1000000, 500000, 250000, 128000, 115200,
  76800, 57600, 38400.
- `?` is NOT implemented; `ID2` address changing is intentionally disabled.
- Moving the actuator requires U/D refreshes within its 500 ms watchdog.
- Actuator direction reversal blocks the MCU loop for 100 ms.
- PING/READ may wait 30 ms per servo; the ROS bridge spaces outgoing frames.
- `S` brakes only the actuator. There is no servo-stop/torque-off command in
  this supplied firmware, and no actuator height feedback.
- `SENT (not confirmed)` is not a servo arrival acknowledgement.

The bridge waits for the v2 HELP response and baud confirmation before
accepting motion. It does not issue PAN/TILT/PT automatically on startup.
