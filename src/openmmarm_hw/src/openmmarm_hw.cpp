#include "openmmarm_hw/openmmarm_hw.hpp"

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openmmarm_hw {

CallbackReturn
OpenMMArmHW::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // 读取参数
  if (info_.hardware_parameters.find("has_gripper") !=
      info_.hardware_parameters.end()) {
    has_gripper_ = info_.hardware_parameters["has_gripper"] == "true";
  }

  num_joints_ = has_gripper_ ? 7 : 6;

  // 连接参数（连接到 openmmarm_controller 的 CmdSdk）
  controller_ip_ = info_.hardware_parameters.count("controller_ip")
                       ? info_.hardware_parameters["controller_ip"]
                       : "127.0.0.1";
  sdk_own_port_ = info_.hardware_parameters.count("sdk_own_port")
                      ? std::stoi(info_.hardware_parameters["sdk_own_port"])
                      : 8072;
  controller_port_ =
      info_.hardware_parameters.count("controller_port")
          ? std::stoi(info_.hardware_parameters["controller_port"])
          : 8871;

  // 初始化状态和命令向量
  hw_states_position_.resize(num_joints_, 0.0);
  hw_states_velocity_.resize(num_joints_, 0.0);
  hw_states_effort_.resize(num_joints_, 0.0);
  hw_commands_position_.resize(num_joints_, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
              "初始化完成 - Controller: %s:%d, 本地端口: %d, 关节数: %zu",
              controller_ip_.c_str(), controller_port_, sdk_own_port_,
              num_joints_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
OpenMMArmHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
              "配置硬件接口，连接到 Controller...");

  // 创建 SDK 客户端
  arm_ = std::make_unique<ArmSdkClient>(controller_ip_, sdk_own_port_,
                                        controller_port_);

  if (!arm_->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenMMArmHW"), "ArmSdkClient 初始化失败");
    return CallbackReturn::ERROR;
  }

  // 首次 sendRecv，获取初始关节位置
  arm_->sendRecv();

  if (arm_->isConnected()) {
    for (size_t i = 0; i < 6; ++i) {
      hw_states_position_[i] = arm_->armState.q[i];
      hw_commands_position_[i] = hw_states_position_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
                "已连接到 Controller，读取了初始位置");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("OpenMMArmHW"),
                "Controller 尚未响应，将在激活后重试");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "硬件配置完成");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
OpenMMArmHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "激活硬件接口...");

  // 向 controller 发送 JointPositionCtrl 模式切换
  arm_->armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::JOINT_CTRL);

  // 读取当前位置作为初始命令
  for (size_t i = 0; i < num_joints_; ++i) {
    hw_commands_position_[i] = hw_states_position_[i];
  }

  // 将初始位置写入目标
  for (size_t i = 0; i < 6; ++i) {
    arm_->armCmd.q_d[i] = hw_commands_position_[i];
  }

  arm_->sendRecv();

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "硬件已激活");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
OpenMMArmHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "停用硬件接口...");

  // 向 controller 发送 Passive 模式切换
  arm_->armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::PASSIVE);
  arm_->sendRecv();

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "硬件已停用");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenMMArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_states_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocity_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &hw_states_effort_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenMMArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_position_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type
OpenMMArmHW::read(const rclcpp::Time & /*time*/,
                  const rclcpp::Duration & /*period*/) {
  // 从 ArmState（controller 回传）读取关节状态
  for (size_t i = 0; i < 6; ++i) {
    hw_states_position_[i] = arm_->armState.q[i];
    hw_states_velocity_[i] = arm_->armState.dq[i];
    hw_states_effort_[i] = arm_->armState.tau[i];
  }

  // 夹爪状态（TODO: 待 controller 支持后启用）
  // if (has_gripper_) {
  //   hw_states_position_[6] = arm_->armState.gripperAngle;
  //   hw_states_velocity_[6] = arm_->armState.gripperSpeed;
  //   hw_states_effort_[6] = arm_->armState.gripperTau;
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenMMArmHW::write(const rclcpp::Time & /*time*/,
                   const rclcpp::Duration & /*period*/) {
  // 将 ros2_control 的命令写入 ArmCmd
  arm_->armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::JOINT_CTRL);

  for (size_t i = 0; i < 6; ++i) {
    arm_->armCmd.q_d[i] = hw_commands_position_[i];
  }

  // 夹爪命令（TODO: 待 controller 支持后启用）
  // if (has_gripper_) {
  //   arm_->armCmd.gripperCmd = hw_commands_position_[6];
  // }

  // 发送指令，接收状态
  arm_->sendRecv();

  return hardware_interface::return_type::OK;
}

// Gripper Action 回调实现
rclcpp_action::GoalResponse OpenMMArmHW::handle_gripper_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const GripperAction::Goal> /*goal*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "收到夹爪控制请求");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OpenMMArmHW::handle_gripper_cancel(
    const std::shared_ptr<GoalHandleGripper> /*goal_handle*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "收到夹爪取消请求");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OpenMMArmHW::handle_gripper_accepted(
    const std::shared_ptr<GoalHandleGripper> goal_handle) {
  // 在新线程中执行夹爪控制
  std::thread{[this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GripperAction::Feedback>();
    auto result = std::make_shared<GripperAction::Result>();

    gripper_command_ = goal->command.position;

    // 简化的夹爪控制逻辑
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      // 检查是否达到目标
      if (std::abs(gripper_position_ - gripper_command_) < 0.01) {
        result->position = gripper_position_;
        result->reached_goal = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "夹爪到达目标位置");
        return;
      }

      // 发布反馈
      feedback->position = gripper_position_;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }
  }}.detach();
}

} // namespace openmmarm_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openmmarm_hw::OpenMMArmHW,
                       hardware_interface::SystemInterface)
