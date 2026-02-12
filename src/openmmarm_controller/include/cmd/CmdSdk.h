#pragma once

#include "openmmarm_sdk/openmmarm_arm_common.h"
#include <memory>

// 前向声明，避免与 CtrlComponents.h 循环依赖
enum class ArmFSMState;

/**
 * @brief 上层指令接口抽象基类
 *
 * 定义了 Controller 与外部 SDK 客户端之间的通信接口。
 * 子类负责实现具体的传输协议（如 UDP、ROS Service 等）。
 *
 * 每个控制周期中，FiniteStateMachine 会调用 sendRecv()
 * 来接收外部指令并回传当前状态。
 */
class CmdSdk {
public:
  CmdSdk() = default;
  virtual ~CmdSdk() = default;

  /**
   * @brief 初始化通信（绑定端口等）
   * @return 是否成功
   */
  virtual bool init() = 0;

  /**
   * @brief 当前是否有客户端连接
   */
  virtual bool isConnected() = 0;

  /**
   * @brief 收发一次数据
   *
   * 接收客户端发来的 ArmCmd，发送当前 ArmState 给客户端。
   * 在控制循环的每一帧中被调用。
   */
  virtual void sendRecv() = 0;

  /**
   * @brief 获取 SDK 客户端请求的目标 FSM 状态
   * @return 目标 ArmFSMState 枚举值
   */
  ArmFSMState getTargetFSMState() const;

  // SDK 指令（由 sendRecv() 填充）
  OPENMMARM_SDK::ArmCmd armCmd{};

  // 机械臂状态（需在调用 sendRecv() 前填充）
  OPENMMARM_SDK::ArmState armState{};

  // 协议版本号
  static constexpr std::array<uint8_t, 3> PROTOCOL_VERSION = {{1, 0, 0}};
};
