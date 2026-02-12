#pragma once

/**
 * @file math_types.h
 * @brief Eigen 类型别名定义
 *
 * 提供常用的向量和矩阵类型别名，方便在 SDK 中使用。
 */

#include <eigen3/Eigen/Dense>
#include <vector>

// 向量类型
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using VecX = Eigen::Matrix<double, Eigen::Dynamic, 1>;

// 四元数
using Quat = Eigen::Matrix<double, 4, 1>;

// 矩阵类型
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat4 = Eigen::Matrix<double, 4, 4>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using RotMat = Eigen::Matrix<double, 3, 3>;
using HomoMat = Eigen::Matrix<double, 4, 4>;
using MatX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

// 工具函数
template <typename T> inline VecX stdVecToEigenVec(const T &stdVec) {
  VecX eigenVec = Eigen::VectorXd::Map(&stdVec[0], stdVec.size());
  return eigenVec;
}

inline std::vector<double> eigenVecToStdVec(const VecX &eigenVec) {
  return std::vector<double>(eigenVec.data(),
                             eigenVec.data() + eigenVec.size());
}
