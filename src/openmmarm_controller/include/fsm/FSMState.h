#pragma once

#include "ctrl/CtrlComponents.h"
#include <memory>
#include <string>

// FSM 模式定义
enum class FSMMode { NORMAL, CHANGE, EXIT };

// 状态索引类型定义 (避免与系统 mode_t 冲突)
using FSMStateMode = int;

// 状态枚举
enum class ArmFSMState {
  INVALID = -1,
  PASSIVE = 0,
  JOINT_CTRL = 1,
  CARTESIAN = 2,
  TEACH = 3,
  TEACH_REPEAT = 4,
  CALIBRATION = 5,
  LOW_CMD = 6
};

/**
 * @brief FSM 状态基类
 *
 * 所有控制状态都继承自此类，实现统一的状态接口。
 */
class FSMState {
public:
  FSMState(std::shared_ptr<CtrlComponents> ctrlComp, ArmFSMState stateEnum,
           const std::string &stateName);

  virtual ~FSMState() = default;

  /**
   * @brief 进入状态时调用
   */
  virtual void enter() = 0;

  /**
   * @brief 状态主循环
   */
  virtual void run() = 0;

  /**
   * @brief 退出状态时调用
   */
  virtual void exit() = 0;

  /**
   * @brief 检查状态切换
   * @return 下一个状态的枚举值
   */
  virtual ArmFSMState checkChange() { return stateEnum_; }

  // 获取状态信息
  ArmFSMState getStateEnum() const { return stateEnum_; }
  std::string getStateName() const { return stateName_; }

protected:
  std::shared_ptr<CtrlComponents> ctrlComp_;
  ArmFSMState stateEnum_;
  std::string stateName_;
};
