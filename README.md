# OpenMMArm-SDK

OpenMMArm 的 ROS 2 工作空间，包含控制器、`ros2_control` 硬件接口、MoveIt 配置、SDK 和一键启动包。

## 架构简述

项目分为四层：

1. 应用层：MoveIt / 自定义节点 / SDK 用户程序
2. 接入层：`openmmarm_hw`（`ros2_control`）和 `openmmarm_sdk`（UDP 客户端）
3. 控制层：`openmmarm_controller`（FSM + 安全逻辑 + IO）
4. 执行层：仿真 Topic 或真机 MCU（UDP）

详细架构请看：`OPENMMARM_ARCHITECTURE.md`

## 快速使用

### 1. 编译

```bash
colcon build
source install/setup.bash
```

### 2. 仿真一键启动

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

常用参数：

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py use_rviz:=true use_moveit:=true
```

### 3. 真机启动

先启动控制器（UDP 模式）：

```bash
ros2 run openmmarm_controller openmmarm_ctrl --ros-args \
  -p communication:=UDP \
  -p udp.mcu_ip:=192.168.123.110 \
  -p udp.mcu_port:=8881
```

再启动 bringup：

```bash
ros2 launch openmmarm_bringup real_arm.launch.py use_rviz:=true
```

### 4. 运行 SDK 示例

```bash
./install/openmmarm_sdk/lib/openmmarm_sdk/example_joint_ctrl
./install/openmmarm_sdk/lib/openmmarm_sdk/example_lowcmd
```

## 各包入口

- `src/openmmarm_bringup/README.md`：统一启动（仿真/真机）
- `src/openmmarm_controller/README.md`：控制器使用与参数
- `src/openmmarm_hw/README.md`：`ros2_control` 硬件接口使用
- `src/openmmarm_moveit_config/README.md`：MoveIt 配置与启动
- `src/openmmarm_sdk/README.md`：SDK API 与示例
