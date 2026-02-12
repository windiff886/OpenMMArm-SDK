#ifndef OPENMMARM_HW_HPP
#define OPENMMARM_HW_HPP

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "openmmarm_hw/arm_sdk_client.hpp"

namespace openmmarm_hw {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GripperAction = control_msgs::action::GripperCommand;
using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperAction>;

/**
 * @brief OpenMMARM 硬件接口类
 *
 * 基于 ros2_control 的 SystemInterface，作为 openmmarm_controller 的 SDK
 * 客户端。 通过 UDP（ArmCmd/ArmState）与 controller 的 CmdSdk 通信。
 *
 * 架构对齐 z1_hw：hw 不直接控制硬件，而是通过 SDK 连接到 controller，
 * 由 controller 的 FSM、安全层负责实际的硬件通信。
 */
class OpenMMArmHW : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OpenMMArmHW)

  /**
   * @brief 初始化：读取 URDF 中的硬件参数
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  /**
   * @brief 配置：创建 ArmSdkClient 并连接到 controller
   */
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief 激活：向 controller 发送 JointPositionCtrl 模式切换
   */
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief 停用：向 controller 发送 Passive 模式切换
   */
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief 导出关节状态接口 (position, velocity, effort)
   */
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  /**
   * @brief 导出关节命令接口 (position)
   */
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /**
   * @brief 从 controller 回传的 ArmState 中读取关节状态
   */
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  /**
   * @brief 将关节目标写入 ArmCmd 并发送给 controller
   */
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // 关节数量（默认 6，有夹爪则 7）
  size_t num_joints_{6};
  bool has_gripper_{false};

  // 状态变量（从 ArmState 读取）
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  // 命令变量（写入 ArmCmd）
  std::vector<double> hw_commands_position_;

  // SDK 客户端（连接 openmmarm_controller 的 CmdSdk）
  std::unique_ptr<ArmSdkClient> arm_;

  // 连接参数
  std::string controller_ip_;
  int sdk_own_port_;
  int controller_port_;

  // Gripper Action Server
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<GripperAction>::SharedPtr gripper_action_server_;

  // Gripper 状态
  double gripper_position_{0.0};
  double gripper_velocity_{0.0};
  double gripper_effort_{0.0};
  double gripper_command_{0.0};

  // Gripper Action 回调
  rclcpp_action::GoalResponse
  handle_gripper_goal(const rclcpp_action::GoalUUID &uuid,
                      std::shared_ptr<const GripperAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_gripper_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle);

  void
  handle_gripper_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);
};

} // namespace openmmarm_hw

#endif // OPENMMARM_HW_HPP
