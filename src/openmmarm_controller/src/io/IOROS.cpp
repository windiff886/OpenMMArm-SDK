#include "io/IOROS.h"
#include <functional>

IOROS::IOROS(rclcpp::Node::SharedPtr node) : node_(node) {}

bool IOROS::init() {
  initPublishers();
  initSubscribers();

  // 等待连接建立
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  is_connected_ = true;
  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "IOROS 初始化完成");
  return true;
}

void IOROS::initPublishers() {
  for (size_t i = 0; i < 6; ++i) {
    std::string topic = "/openmmarm/joint" + std::to_string(i) + "/command";
    joint_cmd_pubs_[i] =
        node_->create_publisher<openmmarm_interfaces::msg::MotorCmd>(topic, 10);
  }
}

void IOROS::initSubscribers() {
  auto create_callback = [this](size_t idx) {
    return
        [this, idx](const openmmarm_interfaces::msg::MotorState::SharedPtr msg) {
          joint_states_[idx] = *msg;
        };
  };

  for (size_t i = 0; i < 6; ++i) {
    std::string topic = "/openmmarm/joint" + std::to_string(i) + "/state";
    joint_state_subs_[i] =
        node_->create_subscription<openmmarm_interfaces::msg::MotorState>(
            topic, 10, create_callback(i));
  }
}

bool IOROS::sendRecv(const LowLevelCmd *cmd, LowLevelState *state) {
  sendCmd(cmd);
  recvState(state);
  return true;
}

void IOROS::sendCmd(const LowLevelCmd *cmd) {
  for (size_t i = 0; i < 6; ++i) {
    openmmarm_interfaces::msg::MotorCmd msg;
    msg.mode = cmd->mode[i];
    msg.q = cmd->q[i];
    msg.dq = cmd->dq[i];
    msg.tau = cmd->tau[i];
    msg.kp = cmd->kp[i];
    msg.kd = cmd->kd[i];

    joint_cmd_pubs_[i]->publish(msg);
  }
}

void IOROS::recvState(LowLevelState *state) {
  for (size_t i = 0; i < 6; ++i) {
    state->mode[i] = joint_states_[i].mode;
    state->q[i] = joint_states_[i].q;
    state->dq[i] = joint_states_[i].dq;
    state->ddq[i] = joint_states_[i].ddq;
    state->tau_est[i] = joint_states_[i].tau_est;
    state->temperature[i] = joint_states_[i].temperature;
  }
}

bool IOROS::isConnected() { return is_connected_; }
