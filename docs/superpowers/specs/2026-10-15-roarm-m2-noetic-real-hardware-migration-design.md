# RoArm-M2-S ROS1 Noetic 实机控制迁移设计

## 1. 目标

本设计只处理 XZB 项目的机械臂部分。目标不是继续修补现有实验性控制链，而是把 RoArm-M2-S 的官方/已验证控制语义迁移为 ROS1 Noetic 可长期使用的机械臂控制栈，并以真实 Orin + RoArm-M2-S 实机运行结果作为最终验收依据。

阶段目标：

1. 机械臂能够稳定启动并读取真实反馈。
2. base / shoulder / elbow / gripper 四个关节能够按 ROS 关节语义双向运动。
3. PARK / HOME / READY 三个命名姿态语义固定且可重复到达。
4. FK / IK 使用与官方 M2 几何模型一致的定义。
5. MoveIt1 能将关节目标/笛卡尔目标规划为真实关节轨迹并驱动实机。
6. 以上完成后再进入 TCP、D435i 手眼标定和 VLA；本阶段不实现 VLA。

## 2. 强制边界

- 运行环境固定为 ROS1 Noetic / catkin，不引入 ROS2 Humble 运行时。
- 只修改机械臂相关代码、机械臂描述、机械臂 MoveIt1 配置和必要的机械臂接口。
- 不修改底盘、导航、D435i、其它相机、VLA、机器人其它业务节点。
- 不重构与机械臂迁移无关的工程目录。
- 不以仿真、fake controller、RViz 动作为“完成”证据。
- 单元测试/离线数学测试只用于防止代码错误；最终通过必须由真实 RoArm-M2-S 实机运动、反馈和到位结果确认。
- 不把上一版的实验性 T1041/Home/坐标补偿逻辑继续叠加到新实现中；确认有价值的基础能力可保留，否则删除或旁路。
- 串口继续保持单 owner；不主动切换 DTR/RTS。

## 3. 源码审计结论

### 3.1 用户提供的 `Roarm_ws_em1.zip`

该包是早期 ROS2 演示工程，核心控制方式为订阅 `JointState` 后直接发送 T102：

```text
joint_states
  -> [base, shoulder, elbow, hand]
  -> T102
  -> serial
  -> RoArm
```

其中 `serial_ctrl_py.py` 的核心发送语义为：base/shoulder/elbow 直接使用消息中的弧度，夹爪做额外角度转换后写入 T102。工程本身标注为 RoArm-M1、使用 ROS2/ament、没有真实反馈闭环、没有 MoveIt 和正式 FK/IK，因此不能原样作为 M2-S 最终控制栈，但“ROS 关节目标直接驱动真实关节控制”的主思路保留。

### 3.2 Waveshare 当前 M2 ROS/SDK 行为

正式迁移以 RoArm-M2 官方模型和 SDK 语义为基准：

- 主关节名称：
  - `base_link_to_link1`
  - `link1_to_link2`
  - `link2_to_link3`
  - `link3_to_gripper_link`
- 官方 ROS driver 从 `JointState` 按关节名提取弧度并调用 `joints_radian_ctrl(...)`。
- 对 M2 的 base / shoulder / elbow 不做额外符号反转。
- angular-direct 夹爪在 SDK/wire 层执行 `hand = pi - gripper` 语义转换。
- 官方 M2 URDF 四个 revolute joint 均使用与官方模型一致的正轴定义；`hand_tcp` 的方向按官方模型恢复。
- 官方几何模型已经定义 M2 的尺寸、FK `computePosbyJointRad(...)` 和 IK `computeJointRadbyPos(...)`。
- 官方 MoveIt 配置存在 M2 IKFast solver，可作为 ROS1 MoveIt1 迁移基准。

## 4. 当前 XZB 机械臂链的问题

本轮重构将下列内容视为需要替换的实验实现，而不是继续兼容的既定接口：

1. `joint_mapping.py` 对 base / shoulder 的负号映射与官方当前 M2 语义不一致。
2. `xzb_description` 为配合该映射又反转 base / shoulder 的 URDF axis，形成双重补偿。
3. 当前 `hand_tcp` 方向与官方 M2 模型不一致，不能用于后续 TCP/手眼标定。
4. T1041 被提升成真实笛卡尔运动主路径，导致“回原 XYZ 不等于回原关节姿态”，并可能选择不同 IK 构型。
5. 当前 production FK/IK 为自写数值求解，和官方 M2 solver / IKFast 存在两套运动学定义。
6. PARK/HOME/READY 的语义未与官方命名姿态及用户要求分离。
7. 运动层叠加了较多诊断/安全实验逻辑，影响“先稳定控制真实机械臂”的主线。

## 5. 新控制架构

```text
CLI / MoveIt1 / later VLA
           |
           v
   xzb_roarm_control
   - PARK / HOME / READY
   - joint target
   - Cartesian target -> IK
   - FK / IK
           |
           v
      joint radians
           |
           v
   xzb_roarm_driver
   - T102: 主多关节目标
   - T101: 单关节调试/必要的单轴命令
   - T105: 真实反馈
   - SerialTransport: 唯一串口 owner
           |
           v
      RoArm-M2-S
```

关键原则：

- `/joint_states` 只表达真实/标准关节状态，不再同时承担“控制命令 topic”的双重职责。
- MoveIt1、CLI、后续 VLA 都先形成标准 ROS 关节目标，再由 driver 转为 RoArm 协议。
- T1041 可以保留为厂商直接姿态接口/诊断能力，但不作为 MoveIt/VLA 的正式主控制路径。
- 一个运动学定义贯穿 URDF、FK、IK、MoveIt、TF、后续手眼标定。

## 6. 关节语义

ROS 与控制器的正式语义固定如下：

```text
ROS base      -> controller base      原值
ROS shoulder  -> controller shoulder  原值
ROS elbow     -> controller elbow     原值
ROS gripper   -> controller hand      pi - q  （angular-direct）
```

不再为了适配旧代码对 base / shoulder 进行额外取负。

关节限制以官方 M2 URDF/MoveIt 配置为基准迁移，再结合实机可达范围核对；禁止用错误坐标映射去“补偿”限制。

## 7. 机械臂模型

`xzb_description` 的 RoArm 部分恢复官方 M2 的几何和坐标定义：

- 使用 M2 正确 link/joint origin。
- base / shoulder / elbow / gripper 的 axis 与官方一致。
- `hand_tcp` 固定变换与官方 M2 模型一致。
- 保留 `tcp_calibrated`、`arm_camera_mount_link` 作为后续标定挂点，但默认值继续代表“未标定”。
- 不在本阶段修改底盘到机械臂底座的系统级安装外参，除非现有 XZB 机械安装已经有明确测量值。

## 8. PARK / HOME / READY

三个姿态严格分离：

### PARK

用户要求的任务结束姿态：两段主杆尽量拉成直线，并整体竖直/与底座平面近似垂直。

```text
base     = 0
shoulder = 0
elbow    = 0
```

PARK 用于开机整理姿态和任务结束停车，不作为笛卡尔规划工作姿态。

### HOME

遵循官方 M2 named state：

```text
base     = 0
shoulder = 0
elbow    = 1.5708
```

### READY / WORK

遵循官方 M2 MoveIt ready 基准：

```text
base     = 0
shoulder = 0
elbow    = 2.618
```

READY 用于 MoveIt、标定和后续抓取/VLA 的工作起点。实机确认后如因实际安装空间需要微调，只允许通过机械臂配置文件调整，不硬编码散落在程序中。

## 9. Driver 设计

保留当前 `SerialTransport` 的有价值能力：

- `/dev/ttyUSB0`、115200 作为当前机器默认参数，但必须参数化。
- 单串口 owner。
- 串行化完整 JSON frame 写入。
- 写后 flush。
- 断线重连。
- 不切换 DTR/RTS。

重写/整理协议层使其与官方 M2 SDK 行为一致：

- T102：正式多关节控制主路径。
- T101：单关节调试/单轴命令。
- T105/T1051：真实位置/关节/负载反馈。
- speed / acceleration 采用官方参数语义，不能继续作为固定的实验常量散落在节点里。
- 夹爪转换只存在一个权威位置，避免 ROS 层和 wire 层重复转换。

## 10. FK / IK / MoveIt1

正式运动学必须与官方 M2 定义一致。

- FK：迁移/复现官方 `computePosbyJointRad(...)` 几何定义，并用官方给定尺寸作为唯一基准。
- IK：优先将官方 M2 IKFast solver / plugin 迁移到 ROS1 MoveIt1；必要时保留官方解析几何 IK 作为服务和回归参考。
- 当前自写 DLS IK 不再作为 production solver；可以保留在非默认实验/测试位置，或在确认无调用后删除。
- MoveIt1 规划 group 为 M2 的 `hand` 三轴主臂，gripper 独立。
- MoveIt1 轨迹执行输出标准 joint trajectory，最终通过 ROS1 action/controller bridge 转成真实 T102 关节目标。
- 笛卡尔目标流程为 `XYZ -> IK/MoveIt -> joint trajectory -> T102`，不是 `XYZ -> T1041`。

## 11. 实机验收顺序

任何阶段只有 Orin + 实际 RoArm-M2-S 返回真实结果才能升级状态。

### H1 启动与反馈

- ROS1 Noetic 启动成功。
- `/roarm/connected = True`。
- 能持续得到四关节真实反馈和末端位置反馈。
- 启动本身不产生非预期运动。

### H2 PARK

- 从当前非零姿态执行 PARK。
- 实物两段主杆拉直并竖直。
- 反馈关节角接近 PARK 目标。
- 重复执行结果一致。

### H3 四关节方向

逐轴做明确可见但可控的往返动作：

- base 正/反方向。
- shoulder 正/反方向。
- elbow 正/反方向。
- gripper 开/合。

验收重点是方向、幅度、非目标轴不应出现明显异常运动、返回姿态可重复。

### H4 FK/IK 实机闭环

- 读取真实 joint state。
- FK 算出 TCP。
- 对可达目标做 IK。
- 将 IK 结果通过真实关节控制发送到机械臂。
- 读取真实反馈并比较 joint/TCP 到位误差。

只打印离线 FK/IK 数值不算通过。

### H5 MoveIt1 实机

- RViz/MoveIt1 目标能生成规划。
- FollowJointTrajectory 真正驱动 RoArm-M2-S。
- 实机按轨迹到达目标。
- 规划结果与真实 `/joint_states`、TF 一致。

完成 H1-H5 后，机械臂控制阶段才算可用于后续标定。

## 12. 明确不做的事情

本轮不做：

- VLA 模型接入。
- D435i 手眼标定。
- 抓取策略。
- 底盘控制改造。
- 导航改造。
- 机械臂 PID / EEPROM / torque limit 调参，除非实机迁移后出现明确且可复现的底层问题，并有单独诊断结论。
- 为了“看起来更安全”继续堆叠与当前可运行目标无关的控制门和协议实验。

## 13. 修改范围白名单

实施阶段默认只允许修改以下机械臂路径；发现必须越界时先停止并说明原因，不直接改：

```text
catkin_ws/src/xzb/xzb_roarm_driver/**
catkin_ws/src/xzb/xzb_roarm_control/**
catkin_ws/src/xzb/xzb_description/**
catkin_ws/src/xzb/xzb_roarm_moveit_config/**
catkin_ws/src/xzb/xzb_interfaces/**   # 仅当机械臂接口确有必要变化

docs/validation/ROARM_*
docs/superpowers/specs/*roarm*
docs/superpowers/plans/*roarm*
```

其它路径视为禁止修改。

## 14. 完成标准

本重构不以“代码已提交”“单元测试通过”“RViz 能动”“FK/IK 离线算通”为完成。

只有以下条件同时成立才算机械臂迁移完成：

1. H1-H5 均由用户在真实 Orin + RoArm-M2-S 上执行并提供通过结果。
2. PARK 能稳定恢复到用户要求的竖直拉直停车姿态。
3. 四关节方向和关节语义与实体一致。
4. FK/IK 与真实运动结果一致到可接受误差。
5. MoveIt1 能真实驱动机械臂完成规划动作。
6. 未修改白名单以外的 XZB 模块。

完成后才进入 TCP / D435i 手眼标定 / VLA 阶段。
