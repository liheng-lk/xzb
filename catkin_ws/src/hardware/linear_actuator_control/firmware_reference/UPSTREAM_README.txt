STM32F103RCT6: dual Feetech ST3215 (ID1=PAN, ID2=TILT) + original linear actuator.

WIRING
- PC CP2102 TXD -> STM32 PA10 / USART1_RX
- PC CP2102 RXD -> STM32 PA9 / USART1_TX
- CP2102 GND -> STM32 GND
- STM32 PA2 / USART2_TX -> Waveshare Bus Servo Adapter (A) TX
- STM32 PA3 / USART2_RX -> Waveshare Bus Servo Adapter (A) RX
- STM32 GND -> Waveshare GND
- Both Waveshare jumpers: A (UART mode), not B
- External 12V+/- -> Waveshare power + and - (servo power)
- ST3215 servo ID1 and ID2 -> two Waveshare D/V/G sockets
- Shared ground is mandatory. NEVER drive servos from STM32 3.3V/5V.

INSTALL SOURCE (on Ubuntu, from your Downloads directory)
cd ~/stm32_ws/linear_actuator_stm32
cp Src/main.c Src/main.c.bak
cp "$(xdg-user-dir DOWNLOAD)/stm32_dual_servo_main.c" Src/main.c
cmake --build build/Debug --clean-first
cube programmer -c port=SWD freq=950 -d ~/stm32_ws/linear_actuator_stm32/build/Debug/linear_actuator_stm32.elf -v -rst

IMPORTANT: the target must be connected before programming. No STM32 target found
means no new firmware was written. Test SWD wiring/board power first.

PC SERIAL (close other serial monitor programs first)
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200 --eol LF --echo

COMMANDS (Enter after each)
HELP
BAUD 1000000       (select matching baud of both servos)
BAUD 115200        (alternative if both configured at 115200)
PING               (expect ID1: OK and ID2: OK)
READ               (read position to choose nearby safe angles)
PAN 180            (ID1)
TILT 180           (ID2)
PT 180 180         (both)
U D S F            (original actuator immediate controls)

The previous ID2-changing command has been disabled. Motor commands report SENT
not confirmed until the servo actually responds/moves. The BAUD setting is not
persistent across resets; default on boot is 1000000.
