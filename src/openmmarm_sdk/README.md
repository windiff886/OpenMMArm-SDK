# openmmarm_sdk

OpenMMArm 的 C++ SDK，提供 `ArmCmd`/`ArmState` UDP 通信接口，用于在上位机程序中直接控制机械臂。

## 架构简述

`openmmarm_sdk` 位于“应用接入层”：

- 作为 SDK 客户端连接 `openmmarm_controller` 的 `udp.sdk_port`（默认 `8871`）
- 每个控制周期执行一次 `sendRecv()` 完成“发指令 + 收状态”
- 不直接访问电机，最终执行由 `openmmarm_controller` 决定

## 使用方法

### 1. 编译

```bash
colcon build --packages-select openmmarm_sdk --symlink-install
source install/setup.bash
```

### 2. 先启动控制器

仿真场景推荐：

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

### 3. 运行示例

```bash
./install/openmmarm_sdk/lib/openmmarm_sdk/example_joint_ctrl
./install/openmmarm_sdk/lib/openmmarm_sdk/example_lowcmd
```

可指定控制器 IP：

```bash
./install/openmmarm_sdk/lib/openmmarm_sdk/example_joint_ctrl 192.168.123.110
```

### 4. 最小调用流程

```cpp
#include "openmmarm_sdk/openmmarm_arm.h"

int main() {
  OPENMMARM_SDK::OpenMMArmSdk arm("127.0.0.1");
  arm.init();

  OPENMMARM_SDK::Timer timer(arm.dt);
  while (true) {
    arm.armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::JOINT_CTRL);
    arm.armCmd.q_d[0] = 1.0;
    arm.sendRecv();
    timer.sleep();
  }
}
```

## 常用字段

### `ArmCmd`

- `mode`：目标模式（`PASSIVE` / `JOINT_CTRL` / `LOW_CMD` 等）
- `q_d[6]`：目标关节角
- `dq_d[6]`：目标关节速度
- `tau_d[6]`：前馈力矩
- `Kp[6]`、`Kd[6]`：增益（用于需要自定义增益的场景）

### `ArmState`

- `mode`：当前模式
- `q[6]`、`dq[6]`、`tau[6]`：反馈状态
- `errors[8]`：错误标志
