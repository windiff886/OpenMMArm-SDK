#pragma once

#include "io/IOInterface.h"
#include "openmmarm_interfaces/msg/motor_cmd.hpp"
#include "openmmarm_interfaces/msg/motor_state.hpp"
#include <rclcpp/rclcpp.hpp>

/**
 * @brief ROS 2 接口实现
 *
 * 通过 ROS 2 Topics 与 Gazebo 仿真环境通信。
 */
class IOROS : public IOInterface {
public:
  explicit IOROS(rclcpp::Node::SharedPtr node);
  ~IOROS() override = default;

  bool init() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
  bool isConnected() override;

private:
  rclcpp::Node::SharedPtr node_;

  // 发布器 (控制指令)
  std::array<rclcpp::Publisher<openmmarm_interfaces::msg::MotorCmd>::SharedPtr, 6>
      joint_cmd_pubs_;

  // 订阅器 (状态反馈)
  std::array<
      rclcpp::Subscription<openmmarm_interfaces::msg::MotorState>::SharedPtr, 6>
      joint_state_subs_;

  // 缓存的关节状态
  std::array<openmmarm_interfaces::msg::MotorState, 6> joint_states_;

  bool is_connected_ = false;

  void initPublishers();
  void initSubscribers();
  void sendCmd(const LowLevelCmd *cmd);
  void recvState(LowLevelState *state);
};
