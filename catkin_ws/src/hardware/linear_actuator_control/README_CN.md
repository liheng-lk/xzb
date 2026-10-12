# linear_actuator_control（推杆 + ST3215 双舵机）

本版只替换原工程的 `linear_actuator_control/` 功能包。保留原包名、节点名、launch 文件名和推杆话题，其他功能包和平台配置不变。

对接的固件是你上传的 `STM32_ST3215_dual_servo_v2(1).zip`：

- 推杆：伸出、缩回、刹车停止、自由停止。
- PAN：ST3215 ID1；TILT：ST3215 ID2。
- PT：同步下发两个舵机的目标角度。
- 推杆运行时可发送舵机命令，同一节点统一管理串口发送和 U/D 看门狗刷新。

**不包含新的自动巡检动作序列、Web 舵机按钮或推杆高度闭环定位。** `camera_mast_manager`、前后端、导航和机械臂包均未修改。

## 1. 替换与编译

将本文件夹替换原工程同名目录。不要在同一个 catkin 工作空间的 src 下留两份同名功能包；旧包备份应放在工作空间以外。

以 `~/catkin_ws/src/linear_actuator_control` 为例：

```bash
sudo apt install python3-serial
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果你的工作空间不是 `~/catkin_ws`，替换为实际路径。项目原来的 ROS1 构建流程可以继续使用。

## 2. STM32 固件

`firmware_reference/main.c` 与上传压缩包中的 `Src/main.c` 字节一致，没有改动底层控制。

**已经烧录这一版 V2 固件的板子，无需重复更换固件。** 如果还是旧的推杆固件，先按 `firmware_reference/README.md` 的说明替换你现有 STM32 工程的 `Src/main.c`，再编译和烧录。

上传固件只有 `main.c` 和说明，不是完整 STM32 工程；catkin 不会编译或烧录 STM32 固件。

## 3. 接线（按上传固件说明）

```text
CP2102 TXD -> STM32 PA10 / USART1_RX
CP2102 RXD <- STM32 PA9  / USART1_TX
CP2102 GND <-> STM32 GND

STM32 PB6 -> H-bridge PWML1
STM32 PB7 -> H-bridge PWML2
STM32 PB8 -> H-bridge EN

STM32 PA2 / USART2_TX -> Waveshare Bus Servo Adapter (A) TX
STM32 PA3 / USART2_RX -> Waveshare Bus Servo Adapter (A) RX
STM32 GND <-> Waveshare GND
Both adapter jumpers: A / UART
External 12 V -> adapter servo power
ID1 = PAN; ID2 = TILT; D/V/G servo sockets
```

保持共地。舵机使用外部 12 V 电源，不能从 STM32 3.3 V/5 V 或 CP2102 供电。转接板 TX/RX 标签的连接方式按上传固件说明保留。

## 4. 启动

使用原来的固定设备名：

```bash
roslaunch linear_actuator_control linear_actuator.launch
```

或直接指定 CP2102：

```bash
roslaunch linear_actuator_control linear_actuator.launch port:=/dev/ttyUSB0
```

如果两只舵机都配置为 115200，使用：

```bash
roslaunch linear_actuator_control linear_actuator.launch port:=/dev/ttyUSB0 servo_baudrate:=115200
```

电脑到 STM32 的 `baudrate` 固定为 **115200**；STM32 到舵机的 `servo_baudrate` 默认 **1000000**。两者不要混淆。`BAUD` 只设置 STM32 的 UART2，不会修改舵机 EEPROM。

新开终端查看状态：

```bash
rostopic echo /linear_actuator/status
```

节点先发送推杆停止命令，再识别 V2 HELP 响应并确认 UART2 波特率。看到 `CONTROLLER_READY:SERVOS_NOT_HOMED` 或 `HOST:READY=true` 后才发送运动命令。这个就绪状态只确认 STM32 通信，舵机是否在线还需使用 `PING`。

不要同时用 miniterm、其他串口工具或第二个节点打开同一个 CP2102。原 udev 脚本保持不变：

```bash
cd ~/catkin_ws/src/linear_actuator_control
sudo usermod -aG dialout "$USER"
bash script/udev/install_udev_rule.sh /dev/ttyUSB0
```

新增用户组权限后需重新登录；设备规则安装后重新插拔 CP2102。

## 5. 推杆与舵机控制

统一控制终端：

```bash
rosrun linear_actuator_control actuator_servo_console.py
```

逐行输入命令并按回车：

```text
PING
READ
PAN 180
TILT 180
PT 180 180
U
S
D
S
```

**180 只是命令格式示例，不是确认过的安全角度。** 先 `READ` 读当前位置，根据相机支架和线材行程小幅测试。启动时不自动回中、回零或移动舵机。

| 命令 | 功能 |
| --- | --- |
| `U` / `extend` | 推杆伸出 |
| `D` / `retract` | 推杆缩回 |
| `S` / `stop` | 推杆刹车停止 |
| `F` / `coast` | 推杆自由停止 |
| `PAN 180` | ID1 目标角度 |
| `TILT 180` | ID2 目标角度 |
| `PT 180 180` | 两个舵机同步下发目标 |
| `PING` | 查询两个舵机的通信响应 |
| `READ` | 读两个舵机的当前位置 |
| `status` / `?` | 显示上位机请求状态，就绪后追加一次舵机 READ |
| `BAUD 1000000` | 设置 STM32 舵机 UART2 波特率 |
| `HELP` | 固件命令说明 |

原来的推杆键盘程序仍可使用：

```bash
rosrun linear_actuator_control actuator_keyboard.py
```

直接使用 ROS 话题：

```bash
rostopic pub -1 /linear_actuator/cmd std_msgs/String "data: 'extend'"
rostopic pub -1 /linear_actuator/cmd std_msgs/String "data: 'stop'"
rostopic pub -1 /linear_actuator/servo_cmd std_msgs/String "data: 'PING'"
rostopic pub -1 /linear_actuator/servo_cmd std_msgs/String "data: 'READ'"
# Only after checking the actual safe angle range:
rostopic pub -1 /linear_actuator/servo_cmd std_msgs/String "data: 'PT 180 180'"
```

`/linear_actuator/cmd` 也可接收 PAN/TILT/PT 命令，不需新开串口节点。所有返回信息仍从 `/linear_actuator/status` 输出。

## 6. 机械角度限位与安全边界

`pan_min_deg`、`pan_max_deg`、`tilt_min_deg`、`tilt_max_deg` 可在 launch 中设置。默认 0～360 只是固件的整数位置命令范围，不代表支架能安全转到全部角度，也不是无限连续旋转命令。本包不臆测实际机械限位。

**`S` 只停推杆，不是舵机急停。** 上传固件没有舵机停止/卸力命令；节点收到 S 或退出时只能取消尚未发出的舵机命令，不能撤销已发出的舵机目标。

节点对正在运动的推杆按默认 10 Hz 刷新 U/D；舵机事务期间通过发送间隔调度避免命令互相干扰。固件的推杆看门狗为 500 ms，上位机连续运动超时默认 20 s（`max_motion_sec`）。上层只发送一次 extend/retract 后，节点会继续刷新，直到收到 stop 或超时；上层不再发布命令并不等于立即停止。

串口断线、STM32 复位或收到 WATCHDOG STOP 后，节点清除旧运动请求，不自动恢复运动。请重新确认现场状态后再发新命令。

`SENT (not confirmed)` 只表示固件已发出舵机指令，不能当成已到位。`READ` 只读舵机位置，没有推杆高度反馈。旧的 `?` 已转为上位机查询逻辑，不再向新固件串口发送 `?`。

## 7. 离线检查

```bash
cd ~/catkin_ws/src/linear_actuator_control
python3 -m unittest discover -s tests -v
```

测试使用 ROS/串口模拟对象，不会驱动硬件。它们检查命令协议、超时、限位、断线重连与推杆/舵机交替发送逻辑，不替代真实 ROS1 编译、STM32 烧录和实机联调。
