#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include "auto_aim_interfaces/msg/target.hpp"

namespace auto_aim
{
// 目标状态模型: 封装跟踪器输出的EKF滤波状态，提供预测和装甲板坐标计算
class Target
{
public:
  Target() = default;

  // 从跟踪器的 Target 消息初始化目标状态
  // 状态向量 (11维): [center_x, vx, center_y, vy, z, vz, yaw, v_yaw, r, Δr, dz]
  void from_msg(const auto_aim_interfaces::msg::Target & msg,
                std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now());

  // 返回内部状态向量 (供规划器使用)
  Eigen::VectorXd ekf_x() const;

  // 基于匀速运动学模型前向预测 dt 秒
  void predict(double dt);
  void predict(std::chrono::steady_clock::time_point t);

  // 返回所有装甲板在惯性系下的坐标列表 [(x, y, z, yaw), ...]
  // 考虑装甲板相对机器人中心的旋转半径 r 和高度差 dz
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool isinit = false;

private:
  // 状态向量: [0]center_x, [1]vx, [2]center_y, [3]vy, [4]z, [5]vz,
  //           [6]yaw, [7]v_yaw, [8]r, [9]Δr, [10]dz
  Eigen::VectorXd state_ = Eigen::VectorXd::Zero(11);
  int armor_num_ = 4;                                      // 装甲板数量 (4/2/3)
  std::chrono::steady_clock::time_point t_ = std::chrono::steady_clock::now();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP