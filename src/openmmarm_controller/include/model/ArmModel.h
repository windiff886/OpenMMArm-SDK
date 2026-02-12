#pragma once

/**
 * @brief 机械臂模型占位类
 *
 * 这是一个简化版本的 ArmModel，用于基础控制。
 * 如果需要完整的动力学功能，请安装 pinocchio 并重新实现此类。
 */
class ArmModel {
public:
  ArmModel() = default;
  ~ArmModel() = default;

  // TODO: 如果需要正逆运动学，添加相应方法
  // 例如：
  // Eigen::Vector3d forwardKinematics(const Vec6& q);
  // Vec6 inverseKinematics(const Eigen::Vector3d& pos);
};
