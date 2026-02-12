#include "fsm/State_Passive.h"
#include "cmd/CmdSdk.h"

State_Passive::State_Passive(std::shared_ptr<CtrlComponents> ctrlComp)
    : FSMState(ctrlComp, ArmFSMState::PASSIVE, "Passive") {}

void State_Passive::enter() {
  // 进入被动模式，清零所有指令
  for (int i = 0; i < 6; ++i) {
    ctrlComp_->lowCmd->mode[i] = 0; // 被动模式
    ctrlComp_->lowCmd->q[i] = 0.0f;
    ctrlComp_->lowCmd->dq[i] = 0.0f;
    ctrlComp_->lowCmd->tau[i] = 0.0f;
    ctrlComp_->lowCmd->kp[i] = 0.0f;
    ctrlComp_->lowCmd->kd[i] = 0.0f;
  }
}

void State_Passive::run() {
  // 被动模式不执行任何控制逻辑
  // 保持电机无力矩输出
}

void State_Passive::exit() {
  // 退出被动模式时的清理工作
}

ArmFSMState State_Passive::checkChange() {
  // 从 SDK 接口读取外部指令
  if (ctrlComp_->cmdSdk && ctrlComp_->cmdSdk->isConnected()) {
    ArmFSMState target = ctrlComp_->cmdSdk->getTargetFSMState();
    // Passive 状态允许切换到 JOINT_CTRL
    if (target == ArmFSMState::JOINT_CTRL) {
      return ArmFSMState::JOINT_CTRL;
    }
  }
  return ArmFSMState::PASSIVE;
}
