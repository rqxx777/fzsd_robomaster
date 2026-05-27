#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <vector>

#include "vision_attacker/target.hpp"
#include "tinympc/tiny_api.hpp"
#include "vision_attacker/trajectory.hpp"

namespace auto_aim
{
// MPC 时间参数
constexpr double DT = 0.01;               // 离散时间步长 (秒)
constexpr int HALF_HORIZON = 50;          // 半预测时域长度
constexpr int HORIZON = HALF_HORIZON * 2; // 总预测时域 = 100步 = 1秒

// 轨迹矩阵: 4行 = (yaw 角位置, yaw 角速度, pitch 角位置, pitch 角速度)
using Trajectory = Eigen::Matrix<double, 4, HORIZON>;

// MPC 规划结果
struct Plan
{
  bool control;         // 是否有效控制
  bool fire;            // 是否开火 (MPC跟踪误差 < fire_thresh)
  float target_yaw;     // 目标 Yaw 角 (rad)
  float target_pitch;   // 目标 Pitch 角 (rad)
  float yaw;            // MPC 优化后的 Yaw 角位置 (rad)
  float yaw_vel;        // MPC 优化后的 Yaw 角速度 (rad/s)
  float yaw_acc;        // MPC 优化后的 Yaw 角加速度 (rad/s²)
  float pitch;          // MPC 优化后的 Pitch 角位置 (rad)
  float pitch_vel;      // MPC 优化后的 Pitch 角速度 (rad/s)
  float pitch_acc;      // MPC 优化后的 Pitch 角加速度 (rad/s²)

  Plan()
    : control(false), fire(false), target_yaw(0.0f), target_pitch(0.0f), yaw(0.0f), yaw_vel(0.0f), yaw_acc(0.0f), pitch(0.0f), pitch_vel(0.0f), pitch_acc(0.0f)
  {
  }
};

// 基于 TinyMPC (ADMM) 的双轴独立 MPC 运动规划器
class Planner
{
public:
  Eigen::Vector4d debug_xyza;  // 调试输出: 最近装甲板的 (x, y, z, yaw)

  // yaw_offset/pitch_offset: 输出角度偏移补偿
  // fire_thresh: 开火角度误差阈值
  // decision_speed: 高/低速延迟切换速度阈值
  // max_yaw_acc/max_pitch_acc: 云台最大角加速度约束
  // Q/R: MPC 状态/控制权重矩阵
  Planner(
    double yaw_offset,
    double pitch_offset ,
    double fire_thresh ,
    double low_speed_delay_time ,
    double high_speed_delay_time ,
    double decision_speed,

    double max_yaw_acc,
    double max_pitch_acc,
    std::vector<double> Q_yaw_v,
    std::vector<double> R_yaw_v,
    std::vector<double> Q_pitch_v,
    std::vector<double> R_pitch_v,
    double lag_time, double air_k
  );

  // 规划主函数: 弹道解算 → 轨迹生成 → MPC求解 → 输出 Plan
  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

  TinySolver * yaw_solver_;    // Yaw 轴 MPC 求解器
  TinySolver * pitch_solver_;  // Pitch 轴 MPC 求解器

  void setup_yaw_solver(double max_yaw_acc_ , std::vector<double> Q_yaw_ , std::vector<double> R_yaw_);
  void setup_pitch_solver(double max_pitch_acc_ , std::vector<double> Q_pitch_ , std::vector<double> R_pitch_);

  double lag_time_;    // 延迟时间补偿
  double air_k_;       // 空气阻力系数
  double J_yaw_ = 1.0;
  double J_pitch_ = 1.0;

  // 计算瞄准角度: 根据目标位置+弹道解算，返回 (yaw, pitch)
  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);

  // 生成参考轨迹: 在预测时域内对目标位置序列进行弹道解算
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP