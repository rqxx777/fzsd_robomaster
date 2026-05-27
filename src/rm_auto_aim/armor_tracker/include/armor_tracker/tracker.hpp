// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
// 装甲板数量枚举: 普通=4块, 平衡步兵=2块, 前哨站=3块
enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

// 多目标 EKF 跟踪器: 状态机管理 (LOST/DETECTING/TRACKING/TEMP_LOST/CHANGE_TARGET)
class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff_);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  // 初始化跟踪器，选择距离图像中心最近的装甲板
  void init(const Armors::SharedPtr & armors_msg);

  // 更新跟踪器: EKF 预测 → 数据关联 → EKF 更新 → 状态机转移
  void update(const Armors::SharedPtr & armors_msg);

  // 自适应角速度 (预留接口)
  void adaptAngularVelocity(const double & duration);

  ExtendedKalmanFilter ekf;    // 扩展卡尔曼滤波器

  int tracking_thres;          // 检测→跟踪的确认帧数阈值
  int lost_thres;              // 临时丢失超时帧数阈值
  int change_thres;            // 目标切换确认帧数阈值

  // 跟踪器状态机
  enum State {
    LOST,            // 丢失: 没有跟踪目标
    DETECTING,       // 检测中: 发现目标但尚未确认 (累计 tracking_thres 帧后转入 TRACKING)
    TRACKING,        // 跟踪中: 已确认目标，持续跟踪
    TEMP_LOST,       // 临时丢失: 若干帧未匹配 (超过 lost_thres 帧转入 LOST)
    CHANGE_TARGET,   // 切换目标: 收到切换指令
  } tracker_state;

  std::string tracked_id;          // 当前跟踪目标的数字 ID
  std::string last_tracked_id;     // 上一个跟踪目标的 ID
  Armor tracked_armor;             // 当前跟踪的装甲板
  ArmorsNum tracked_armors_num;    // 跟踪目标的装甲板数量

  double info_position_diff;       // 调试信息: 匹配位置误差
  double info_yaw_diff;            // 调试信息: 匹配角度误差

  Eigen::VectorXd measurement;     // 当前观测向量 (4维: xa, ya, za, yaw)

  Eigen::VectorXd target_state;    // 目标状态向量 (9维): xc,vx,yc,vy,za,vza,yaw,v_yaw,r

  double dz;         // 另一对装甲板的高度差 (4块装甲板的第2/4块)
  double another_r;  // 另一对装甲板的旋转半径

private:
  // 初始化 EKF 状态
  void initEKF(const Armor & a);

  // 目标切换时的初始化
  void initChange(const Armor & armor_msg);

  // 根据装甲板类型更新装甲板数量
  void updateArmorsNum(const Armor & a);

  // 处理装甲板跳变 (旋转目标装甲板快速切换)
  void handleArmorJump(const Armor & a);

  // 四元数转 Yaw 角，处理角度连续性问题
  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  // 从状态向量中提取装甲板的预测位置
  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;      // 最大匹配距离阈值
  double max_match_yaw_diff_;      // 最大匹配角度差阈值

  int detect_count_;               // 检测计数器 (累计到 tracking_thres 后确认跟踪)
  int lost_count_;                 // 丢失计数器 (累计到 lost_thres 后确认丢失)
  int change_count_;               // 切换计数器

  double last_yaw_;                // 上一帧的 Yaw 角 (用于角度连续性)
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
