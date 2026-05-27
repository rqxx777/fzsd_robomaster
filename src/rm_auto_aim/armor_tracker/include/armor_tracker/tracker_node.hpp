// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;

// 装甲板跟踪器 ROS2 节点: 订阅检测结果，发布跟踪目标状态
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  // 检测结果回调: 坐标变换 → 异常值过滤 → 跟踪器更新 → 发布目标
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  // 发布 RViz 可视化标记
  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // XOY 平面内最大允许装甲板距离
  double max_armor_distance_;

  // 帧间时间间隔
  rclcpp::Time last_time_;
  double dt_;
  double lead_time_;         // 预测超前时间 (补偿处理延迟)

  // EKF 过程噪声参数
  double s2qxyz_max_, s2qxyz_min_;    // 位置过程噪声 (速度相关动态调整)
  double s2qyaw_max_, s2qyaw_min_;    // 角度过程噪声
  double s2qr_;                       // 半径过程噪声
  double r_xyz_factor;                // 观测噪声/状态比例因子
  double r_yaw;                       // 角度观测噪声
  double lost_time_thres_;            // 丢失超时时间阈值 (秒)
  std::unique_ptr<Tracker> tracker_;  // 多目标跟踪器

  // 服务: 重置跟踪器 / 切换目标
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_target_srv_;

  // tf2 坐标变换相关
  std::string target_frame_;          // 目标坐标系 (通常为 aim_odom)
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // 发布者
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;  // 调试信息
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;     // 目标状态

  // RViz 可视化标记
  visualization_msgs::msg::Marker position_marker_;    // 位置标记 (球体)
  visualization_msgs::msg::Marker linear_v_marker_;    // 线速度标记 (箭头)
  visualization_msgs::msg::Marker angular_v_marker_;   // 角速度标记 (箭头)
  visualization_msgs::msg::Marker armor_marker_;       // 装甲板标记 (立方体)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
