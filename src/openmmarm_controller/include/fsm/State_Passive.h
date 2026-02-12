#pragma once

#include "fsm/FSMState.h"

/**
 * @brief 被动状态
 *
 * 电机无力矩输出，可用于手动拖拽或待机。
 */
class State_Passive : public FSMState {
public:
  explicit State_Passive(std::shared_ptr<CtrlComponents> ctrlComp);
  ~State_Passive() override = default;

  void enter() override;
  void run() override;
  void exit() override;
  ArmFSMState checkChange() override;
};
