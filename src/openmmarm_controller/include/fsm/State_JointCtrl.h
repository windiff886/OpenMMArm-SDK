#pragma once

#include "fsm/FSMState.h"

/**
 * @brief 关节控制状态
 *
 * 响应上层下发的关节角度/速度/力矩指令。
 */
class State_JointCtrl : public FSMState {
public:
  explicit State_JointCtrl(std::shared_ptr<CtrlComponents> ctrlComp);
  ~State_JointCtrl() override = default;

  void enter() override;
  void run() override;
  void exit() override;
  ArmFSMState checkChange() override;

private:
  void setJointCmd();
};
