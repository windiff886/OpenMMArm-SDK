#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

/**
 * @file openmmarm_arm_common.h
 * @brief OpenMMARM 机械臂 SDK 通信数据结构定义
 *
 * 定义了上层 SDK 客户端与 Controller 之间通信的数据包格式。
 * 通过 UDP 传输，使用 packed 结构确保跨平台二进制兼容。
 */

namespace OPENMMARM_SDK {

/**
 * @brief 机械臂 FSM 模式枚举（SDK 侧使用）
 *
 * 数值与 ArmFSMState 对应，用于通过网络触发 FSM 状态切换。
 */
enum class ArmMode : uint8_t {
  INVALID = 0,
  PASSIVE = 1,
  JOINT_CTRL = 2,
  CARTESIAN = 3,
  TEACH = 4,
  TEACH_REPEAT = 5,
  CALIBRATION = 6,
  LOW_CMD = 7
};

/**
 * @brief 错误类型枚举
 */
constexpr int ERROR_NUM = 8;

enum class ArmError : uint8_t {
  JOINT_POSITION_LIMIT = 0,
  JOINT_VELOCITY_LIMIT = 1,
  COLLISION_DETECTED = 2,
  COMMUNICATION_LOST = 3,
};

/**
 * @brief 上层 SDK 发送给 Controller 的指令包
 *
 * 包含目标 FSM 状态和关节控制指令。
 * 控制律: tau = Kp * (q_d - q) + Kd * (dq_d - dq) + tau_d
 */
#pragma pack(push, 1)
struct ArmCmd {
  // 协议版本号
  std::array<uint8_t, 3> version{};

  // 目标 FSM 状态（映射到 ArmMode 枚举）
  uint8_t mode{};

  // 关节位置增益
  std::array<double, 6> Kp{};

  // 关节速度增益
  std::array<double, 6> Kd{};

  // 目标关节位置 (rad)
  std::array<double, 6> q_d{};

  // 目标关节速度 (rad/s)
  std::array<double, 6> dq_d{};

  // 前馈力矩 (N·m)
  std::array<double, 6> tau_d{};

  // CRC32 校验码
  uint32_t crc{};
};

/**
 * @brief Controller 发送给上层 SDK 的状态包
 *
 * 包含当前 FSM 状态和关节传感器反馈。
 */
struct ArmState {
  // 协议版本号
  std::array<uint8_t, 3> version{};

  // 当前 FSM 状态
  uint8_t mode{};

  // 实测关节位置 (rad)
  std::array<double, 6> q{};

  // 实测关节速度 (rad/s)
  std::array<double, 6> dq{};

  // 实测关节力矩 (N·m)
  std::array<double, 6> tau{};

  // 电机温度 (°C)
  std::array<int8_t, 6> temperature{};

  // 错误状态标志
  std::array<bool, ERROR_NUM> errors{};

  // CRC32 校验码
  uint32_t crc{};
};
#pragma pack(pop)

constexpr std::size_t ARM_CMD_LENGTH = sizeof(ArmCmd);
constexpr std::size_t ARM_STATE_LENGTH = sizeof(ArmState);

} // namespace OPENMMARM_SDK
