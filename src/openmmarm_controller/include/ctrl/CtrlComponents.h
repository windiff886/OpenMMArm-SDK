#pragma once

#include "cmd/CmdSdk.h"
#include "model/ArmModel.h"
#include <Eigen/Dense>
#include <memory>
#include <string>

// 前向声明 (避免循环依赖)
class IOInterface;

// 类型定义
using Vec6 = Eigen::Matrix<double, 6, 1>;

/**
 * @brief FSM 层使用的中间指令结构
 */
struct FsmArmCmd {
  Vec6 q{};     // 目标关节角
  Vec6 qlast{}; // 上一时刻关节角
  Vec6 dq{};    // 目标关节速度
  Vec6 tau{};   // 目标力矩
  Vec6 ftip{};  // 末端力
};

/**
 * @brief 底层控制指令 (发给电机)
 */
struct LowLevelCmd {
  uint8_t mode[6]{}; // 电机模式
  float q[6]{};      // 目标位置
  float dq[6]{};     // 目标速度
  float tau[6]{};    // 目标力矩
  float kp[6]{};     // 刚度
  float kd[6]{};     // 阻尼
};

/**
 * @brief 底层状态反馈 (从电机获取)
 */
struct LowLevelState {
  uint8_t mode[6]{};       // 电机模式
  float q[6]{};            // 当前位置
  float dq[6]{};           // 当前速度
  float ddq[6]{};          // 当前加速度
  float tau_est[6]{};      // 估计力矩
  int8_t temperature[6]{}; // 温度
};

/**
 * @brief 控制组件中枢
 *
 * 作为数据中心，存储所有控制相关的共享数据。
 * 不包含复杂逻辑，只负责数据存储和传递。
 */
struct CtrlComponents {
  CtrlComponents() {
    lowCmd = std::make_shared<LowLevelCmd>();
    lowState = std::make_shared<LowLevelState>();
    fsmArmCmd = std::make_shared<FsmArmCmd>();
  }

  ~CtrlComponents() = default;

  // 控制周期 (默认 250Hz)
  double dt = 0.004;

  // 项目路径
  std::string projectPath;

  // 机械臂动力学模型
  std::shared_ptr<ArmModel> arm;

  // 硬件抽象接口
  std::shared_ptr<IOInterface> ioInter;

  // 上层 SDK 指令接口
  std::shared_ptr<CmdSdk> cmdSdk;

  // 底层指令和状态
  std::shared_ptr<LowLevelCmd> lowCmd;
  std::shared_ptr<LowLevelState> lowState;

  // FSM 使用的中间层指令
  std::shared_ptr<FsmArmCmd> fsmArmCmd;

  // 碰撞检测
  bool collisionOpen = true;
  double collisionLimitT = 10.0;
};
