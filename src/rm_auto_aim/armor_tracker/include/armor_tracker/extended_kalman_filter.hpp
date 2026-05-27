// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{
// 通用扩展卡尔曼滤波器 (Extended Kalman Filter)
// 支持非线性过程模型 f(x) 和非线性观测模型 h(x)
class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  // f: 非线性过程模型 x_{k+1} = f(x_k)
  // h: 非线性观测模型 z_k = h(x_k)
  // j_f: f 的雅可比矩阵
  // j_h: h 的雅可比矩阵
  // u_q: 过程噪声协方差矩阵 (可依赖当前状态)
  // u_r: 观测噪声协方差矩阵 (可依赖观测值)
  // P0: 初始状态误差协方差矩阵
  explicit ExtendedKalmanFilter(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

  // 设置初始状态
  void setState(const Eigen::VectorXd & x0);

  // 预测步骤: 计算先验状态估计 x_pri 和先验误差协方差 P_pri
  Eigen::MatrixXd predict();

  // 更新步骤: 根据测量值 z 计算卡尔曼增益 K 和后验状态估计 x_post
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
  VecVecFunc f;       // 非线性过程模型函数
  VecVecFunc h;       // 非线性观测模型函数
  VecMatFunc jacobian_f;  // 过程模型雅可比
  Eigen::MatrixXd F;  // 过程模型雅可比矩阵 (当前值)
  VecMatFunc jacobian_h;  // 观测模型雅可比
  Eigen::MatrixXd H;  // 观测模型雅可比矩阵 (当前值)

  VecMatFunc update_Q;  // 过程噪声协方差更新函数
  Eigen::MatrixXd Q;    // 过程噪声协方差矩阵
  VecMatFunc update_R;  // 观测噪声协方差更新函数
  Eigen::MatrixXd R;    // 观测噪声协方差矩阵

  Eigen::MatrixXd P_pri;   // 先验误差协方差估计
  Eigen::MatrixXd P_post;  // 后验误差协方差估计

  Eigen::MatrixXd K;    // 卡尔曼增益

  int n;                // 系统状态维度

  Eigen::MatrixXd I;    // n×n 单位矩阵

  Eigen::VectorXd x_pri;   // 先验状态估计
  Eigen::VectorXd x_post;  // 后验状态估计
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
