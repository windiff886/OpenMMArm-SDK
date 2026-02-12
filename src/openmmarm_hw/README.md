# openmmarm_hw

OpenMMArm 的 `ros2_control` 硬件接口插件，实现 `JointTrajectoryController` 与控制器 SDK 通道的桥接。

## 架构简述

`openmmarm_hw` 位于“接入层”：

- 上游接收 `ros2_control` 的关节位置命令
- 下游通过 `ArmSdkClient` 使用 UDP 与 `openmmarm_controller` 通信
- 在 `read()` 中回填状态，在 `write()` 中发送目标 `q_d`

它不直接控制电机，所有命令都先进入 `openmmarm_controller`。

## 使用方法

### 1. 推荐方式（通过 bringup）

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

或真机：

```bash
ros2 launch openmmarm_bringup real_arm.launch.py
```

### 2. 单独启动 `openmmarm_hw`

`openmmarm_hw.launch.py` 依赖 `/robot_description`，通常需要先启动 `robot_state_publisher`：

```bash
ros2 launch openmmarm_description display.launch.py gui:=false
ros2 launch openmmarm_hw openmmarm_hw.launch.py
```

### 3. 运行后检查

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

## 配置入口

### Controller Manager 配置

文件：`config/openmmarm_hw_config.yaml`

主要配置：

- `update_rate`
- `openmmarm_arm_controller` 类型与关节列表
- `joint_state_broadcaster`

### 硬件 UDP 参数

文件：`src/openmmarm_description/urdf/openmmarm.ros2_control.xacro`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `has_gripper` | `false` | 是否启用夹爪关节 |
| `controller_ip` | `127.0.0.1` | `openmmarm_controller` IP |
| `sdk_own_port` | `8072` | 本地 SDK 客户端端口 |
| `controller_port` | `8871` | 控制器 SDK 监听端口 |

## 常见排查

- `openmmarm_hw` 启动后无状态更新：先确认 `openmmarm_controller` 已运行且端口匹配。
- `ros2_control_node` 报找不到 `robot_description`：先启动 `robot_state_publisher`，或改为使用 `openmmarm_bringup`。
