# OpenMMArm 架构详解

本文档只描述 `openmmarm` 相关项目结构与运行机制。

## 1. 总体目标

OpenMMArm 软件栈将“规划、控制、硬件通信”解耦，核心目标是：

- 上层（MoveIt / 自定义程序 / SDK）统一输出关节目标
- 控制层（FSM）集中做模式切换与安全约束
- 执行层支持仿真（ROS Topic）和真机（UDP）两套链路

## 2. 包级架构

| 包名 | 角色 | 关键输入 | 关键输出 |
| --- | --- | --- | --- |
| `openmmarm_bringup` | 系统编排 | launch 参数 | 拉起整套节点 |
| `openmmarm_description` | 模型与 ros2_control 定义 | URDF/Xacro | `/robot_description` |
| `openmmarm_interfaces` | 消息定义 | - | `MotorCmd`/`MotorState` |
| `openmmarm_moveit_config` | 规划配置 | SRDF/YAML | `move_group` 规划服务 |
| `openmmarm_hw` | ros2_control 硬件插件 | 轨迹控制器命令 | `ArmCmd` UDP 请求 |
| `openmmarm_controller` | 控制核心（FSM） | `ArmCmd` / 仿真或真机反馈 | 底层控制命令 / `ArmState` |
| `openmmarm_sdk` | C++ SDK 客户端库 | 用户程序控制指令 | UDP `ArmCmd` / `ArmState` |

## 3. 分层与数据流

```text
[应用层]
MoveIt / 用户节点 / SDK程序
        |
        v
[接入层]
openmmarm_hw (ros2_control) 或 openmmarm_sdk
        |
        v   UDP ArmCmd/ArmState
[控制层]
openmmarm_controller (CmdSdk + FSM + Safety + IO)
        |
        +--> IOROS  -> /openmmarm/jointX/command,state  (仿真)
        |
        +--> IOUDP  -> MCU UDP (真机)
```

### 3.1 仿真链路

1. `move_group` 生成轨迹，交给 `JointTrajectoryController`
2. `openmmarm_hw::write()` 将目标写入 `ArmCmd.q_d`
3. `openmmarm_hw` 通过 UDP 发给 `openmmarm_controller`（CmdSdk）
4. FSM 处理后，经 `IOROS` 发布到 `/openmmarm/jointX/command`
5. 仿真侧回传 `/openmmarm/jointX/state`
6. `openmmarm_controller` 汇总成 `ArmState` 回传 `openmmarm_hw`

### 3.2 真机链路

1. `openmmarm_hw` 仍通过 UDP 向 `openmmarm_controller` 发送 `ArmCmd`
2. `openmmarm_controller` 通过 `IOUDP` 向 MCU 发底层命令包
3. MCU 回传关节状态，控制器转成 `ArmState` 回给 `openmmarm_hw`

## 4. openmmarm_controller 内部架构

### 4.1 主要组件

- `CtrlComponents`：共享数据中枢（`lowCmd`、`lowState`、`fsmArmCmd`）
- `CmdSdk/UdpSdk`：SDK 指令通道（默认端口 `8871`）
- `IOInterface`：下行 IO 抽象
  - `IOROS`：仿真 Topic 收发
  - `IOUDP`：真机 UDP 收发
- `FiniteStateMachine`：状态调度与周期控制

### 4.2 线程模型

`openmmarm_controller` 使用三线程：

1. Main 线程：生命周期管理（约 1Hz）
2. FSM 控制线程：固定周期（`dt=0.004`，250Hz）
3. ROS Executor 线程：处理 ROS 回调（`spin_some(1ms)`）

### 4.3 控制循环顺序（FSM 线程）

每周期执行：

1. `currentState->run()`
2. `currentState->checkChange()` 决定状态切换
3. `ioInter->sendRecv(lowCmd, lowState)`
4. `cmdSdk->sendRecv()`（接收新 `ArmCmd` 并回传 `ArmState`）
5. 休眠到下一个周期

### 4.4 已实现状态

- `PASSIVE`：输出零力矩/零增益
- `JOINT_CTRL`：按 `ArmCmd` 的 `q_d/dq_d/tau_d` 生成底层指令，可动态更新 `Kp/Kd`

## 5. openmmarm_hw 内部架构

`openmmarm_hw` 是 `hardware_interface::SystemInterface` 插件：

- 插件名：`openmmarm_hw/OpenMMArmHW`
- 由 `ros2_control_node` 加载
- 命令接口：`position`
- 状态接口：`position`/`velocity`/`effort`

生命周期行为：

- `on_init`：读取硬件参数（`controller_ip`、`sdk_own_port`、`controller_port`）
- `on_configure`：创建 `ArmSdkClient`，首次同步关节状态
- `on_activate`：请求控制器切换 `JOINT_CTRL`
- `on_deactivate`：切回 `PASSIVE`
- `write`：发送目标关节角到控制器
- `read`：读取控制器回传状态

## 6. 通信接口与端口

### 6.1 SDK 通道（openmmarm_sdk/openmmarm_hw ↔ controller）

- 协议：UDP + CRC32
- 默认端口：`udp.sdk_port = 8871`
- 数据结构定义：`openmmarm_sdk/include/openmmarm_sdk/openmmarm_arm_common.h`
  - `ArmCmd`：`mode/Kp/Kd/q_d/dq_d/tau_d/crc`
  - `ArmState`：`mode/q/dq/tau/temperature/errors/crc`

### 6.2 控制器 ↔ MCU 通道（真机）

- 协议：UDP 二进制包 + CRC32
- 参数：`udp.mcu_ip`、`udp.mcu_port`、`udp.local_port`
- 实现：`openmmarm_controller/src/io/IOUDP.cpp`

### 6.3 控制器 ↔ 仿真通道

- 协议：ROS 2 Topic
- 命令主题：`/openmmarm/jointX/command`
- 状态主题：`/openmmarm/jointX/state`
- 消息类型：`openmmarm_interfaces/msg/MotorCmd`、`MotorState`

## 7. 配置入口与所有权

| 配置文件 | 作用 |
| --- | --- |
| `src/openmmarm_controller/config/openmmarm_controller.yaml` | 控制器运行参数（通信模式、UDP、碰撞） |
| `src/openmmarm_description/urdf/openmmarm.ros2_control.xacro` | `openmmarm_hw` 插件和硬件参数（IP/端口） |
| `src/openmmarm_hw/config/openmmarm_hw_config.yaml` | `controller_manager` 与控制器插件配置 |
| `src/openmmarm_moveit_config/config/*.yaml` | MoveIt 规划/运动学/控制器映射参数 |

## 8. 启动时序

### 8.1 仿真（推荐）

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

时序：

1. 启动 `openmmarm_controller`（默认 ROS 通信）
2. 启动 `robot_state_publisher`
3. 延时启动 `openmmarm_hw`
4. 延时启动 `move_group`（可选）
5. 启动 `rviz2`（可选）

### 8.2 真机

先启动 controller（UDP）：

```bash
ros2 run openmmarm_controller openmmarm_ctrl --ros-args -p communication:=UDP
```

再启动：

```bash
ros2 launch openmmarm_bringup real_arm.launch.py
```
