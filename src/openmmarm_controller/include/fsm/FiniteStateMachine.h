#pragma once

#include "fsm/FSMState.h"
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

/**
 * @brief 有限状态机
 *
 * 管理所有控制状态的调度和切换。
 */
class FiniteStateMachine {
public:
  FiniteStateMachine(std::vector<std::shared_ptr<FSMState>> states,
                     std::shared_ptr<CtrlComponents> ctrlComp);
  ~FiniteStateMachine();

  /**
   * @brief 启动状态机
   */
  void start();

  /**
   * @brief 停止状态机
   */
  void stop();

private:
  void run();

  std::vector<std::shared_ptr<FSMState>> states_;
  std::shared_ptr<CtrlComponents> ctrlComp_;

  std::shared_ptr<FSMState> currentState_;
  std::shared_ptr<FSMState> nextState_;

  FSMMode mode_ = FSMMode::NORMAL;

  std::unique_ptr<std::thread> runThread_;
  std::atomic<bool> running_{false};
};
