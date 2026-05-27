#include "vision_attacker/planner.hpp"
#include "vision_attacker/math_tools.hpp"


using namespace std::chrono_literals;

namespace auto_aim
{
  // 构造函数: 存储偏置/阈值参数，初始化Yaw和Pitch两个独立MPC求解器
Planner::  Planner(
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

  )
{
  yaw_offset_ = yaw_offset;
  pitch_offset_ = pitch_offset;
  fire_thresh_ = fire_thresh;
  low_speed_delay_time_ = low_speed_delay_time;
  high_speed_delay_time_ = high_speed_delay_time;
  decision_speed_ = decision_speed;

  setup_yaw_solver(max_yaw_acc , Q_yaw_v, R_yaw_v);
  setup_pitch_solver(max_pitch_acc , Q_pitch_v, R_pitch_v);
  lag_time_ = lag_time;
  air_k_ = air_k;
}

Plan Planner::plan(Target target, double bullet_speed)
  // 规划主函数: 弹速保护 → 弹道解算求飞行时间 → 前向预测 → 计算参考轨迹 → MPC求解
{
  // 0. Check bullet speed
  if (bullet_speed < 10 ) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  double fly_time = 0.0;
  // Use Tracker_node::solveTrajectory to compute fly_time and pitch (defaults for lagTime and air_k)
  // default lagTime and air_k chosen to match previous usage in vision_attacker
  solveTrajectory(fly_time, min_dist, xyz.z(), lag_time_, bullet_speed, air_k_);
  target.predict(fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    std::cerr << "Unsolvable target " << bullet_speed << std::endl;
    return Plan{};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  //plan.pitch = solveTrajectory(fly_time, min_dist, xyz.z(), lag_time_, bullet_speed, air_k_)*180/M_PI;
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

  // 带延迟的规划: 根据目标角速度选择高/低速延迟时间，预测到未来时刻再规划
Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return Plan{};

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  return plan(*target, bullet_speed);
}
  // Yaw轴 MPC 求解器: 双积分模型 (位置+速度), 加速度约束

void Planner::setup_yaw_solver(double max_yaw_acc_ , std::vector<double> Q_yaw_v_ , std::vector<double> R_yaw_v_)
{
  auto max_yaw_acc = max_yaw_acc_;
  auto Q_yaw = Q_yaw_v_;
  auto R_yaw = R_yaw_v_;

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
  // Pitch轴 MPC 求解器: 与Yaw轴相同的模型结构
}

void Planner::setup_pitch_solver(double max_pitch_acc_ , std::vector<double> Q_pitch_ , std::vector<double> R_pitch_)
{
  auto max_pitch_acc = max_pitch_acc_;
  auto Q_pitch = Q_pitch_;
  auto R_pitch = R_pitch_;

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  // 瞄准角度计算: 选择最近装甲板 → 弹道解算求Pitch → 返回 (yaw+偏置, -pitch-偏置)
  pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  double fly_time = 0.0;
  // 生成参考轨迹: 在预测时域内迭代计算每一步的瞄准角度和角速度
  double traj_pitch = solveTrajectory(fly_time, min_dist, xyz.z(), lag_time_, bullet_speed, air_k_);
  return {tools::limit_rad(azim + yaw_offset_), -traj_pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim