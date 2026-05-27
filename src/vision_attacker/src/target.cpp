#include "vision_attacker/target.hpp"

#include <cmath>
#include "vision_attacker/math_tools.hpp"

namespace auto_aim
{

  // 从跟踪器消息填充内部11维状态向量
void Target::from_msg(const auto_aim_interfaces::msg::Target & msg,
                      std::chrono::steady_clock::time_point t)
{
  // Map message fields into minimal state_ vector:
  // [0]=center_x, [1]=vx, [2]=center_y, [3]=vy, [4]=z, [5]=vz,
  // [6]=angle (yaw), [7]=angular velocity (w), [8]=r, [9]=l (r2-r1), [10]=h (z2 - z1)

  if (state_.size() != 11) state_ = Eigen::VectorXd::Zero(11);

  state_[0] = msg.position.x;
  state_[1] = msg.velocity.x;
  state_[2] = msg.position.y;
  state_[3] = msg.velocity.y;
  state_[4] = msg.position.z;
  state_[5] = msg.velocity.z;
  state_[6] = msg.yaw;
  state_[7] = msg.v_yaw;
  state_[8] = msg.radius_1;
  state_[9] = msg.radius_2 - msg.radius_1;
  state_[10] = msg.dz;

  armor_num_ = msg.armors_num > 0 ? msg.armors_num : 4;
  t_ = t;
  isinit = true;
}

Eigen::VectorXd Target::ekf_x() const { return state_; }

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

  // 匀速运动学前向预测: x += vx*dt, yaw += v_yaw*dt
void Target::predict(double dt)
{
  // simple kinematic prediction
  state_[0] += state_[1] * dt;
  state_[2] += state_[3] * dt;
  state_[4] += state_[5] * dt;
  state_[6] += state_[7] * dt;
  state_[6] = tools::limit_rad(state_[6]);
}

  // 计算所有装甲板在惯性系下的 (x, y, z, yaw) 坐标
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> list;
  const double PI = M_PI;
  for (int i = 0; i < armor_num_; i++) {
    double angle = tools::limit_rad(state_[6] + i * 2.0 * PI / armor_num_);
    bool use_l_h = (armor_num_ == 4) && (i == 1 || i == 3);
    double r = use_l_h ? state_[8] + state_[9] : state_[8];
    double x = state_[0] - r * std::cos(angle);
    double y = state_[2] - r * std::sin(angle);
    double z = use_l_h ? state_[4] + state_[10] : state_[4];
    list.push_back(Eigen::Vector4d{x, y, z, angle});
  }
  return list;
}

}  // namespace auto_aim
