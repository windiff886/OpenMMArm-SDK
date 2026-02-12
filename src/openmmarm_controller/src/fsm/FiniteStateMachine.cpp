#include "fsm/FiniteStateMachine.h"
#include "cmd/CmdSdk.h"
#include "io/IOInterface.h"
#include <chrono>

FiniteStateMachine::FiniteStateMachine(
    std::vector<std::shared_ptr<FSMState>> states,
    std::shared_ptr<CtrlComponents> ctrlComp)
    : states_(std::move(states)), ctrlComp_(ctrlComp) {
  // 默认进入第一个状态 (通常是 Passive)
  if (!states_.empty()) {
    currentState_ = states_[0];
  }
}

FiniteStateMachine::~FiniteStateMachine() { stop(); }

void FiniteStateMachine::start() {
  if (running_)
    return;

  running_ = true;
  if (currentState_) {
    currentState_->enter();
  }

  runThread_ = std::make_unique<std::thread>(&FiniteStateMachine::run, this);
}

void FiniteStateMachine::stop() {
  running_ = false;
  if (runThread_ && runThread_->joinable()) {
    runThread_->join();
  }
}

void FiniteStateMachine::run() {
  while (running_) {
    // 计算控制周期
    auto start = std::chrono::high_resolution_clock::now();

    // 执行当前状态逻辑
    if (currentState_) {
      currentState_->run();

      // 检查状态切换
      ArmFSMState nextStateEnum = currentState_->checkChange();
      if (nextStateEnum != currentState_->getStateEnum()) {
        // 查找下一个状态
        for (auto &state : states_) {
          if (state->getStateEnum() == nextStateEnum) {
            currentState_->exit();
            currentState_ = state;
            currentState_->enter();
            break;
          }
        }
      }
    }

    // IO 通信（与硬件/仿真器）
    if (ctrlComp_->ioInter && ctrlComp_->ioInter->initialized()) {
      ctrlComp_->ioInter->sendRecv(ctrlComp_->lowCmd.get(),
                                   ctrlComp_->lowState.get());
    }

    // SDK 通信（与上层客户端）
    if (ctrlComp_->cmdSdk) {
      // 将当前 FSM 状态写入 armState
      if (currentState_) {
        ctrlComp_->cmdSdk->armState.mode =
            static_cast<uint8_t>(currentState_->getStateEnum());
      }
      ctrlComp_->cmdSdk->sendRecv();
    }

    // 控制周期休眠
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    auto sleep_time =
        std::chrono::microseconds(static_cast<int64_t>(ctrlComp_->dt * 1e6)) -
        elapsed;

    if (sleep_time.count() > 0) {
      std::this_thread::sleep_for(sleep_time);
    }
  }
}
