// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "armor_detector/yolo_detector.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
// #include "vision_interfaces/msg/robot.hpp"

namespace rm_auto_aim
{
// 装甲板检测 ROS2 节点: 订阅相机图像，发布检测到的装甲板列表
class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  // 图像回调: 处理每一帧图像，执行检测流程
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  // 初始化传统CV检测器
  std::unique_ptr<Detector> initDetector();
  // 初始化 YOLO 深度学习检测器
  std::unique_ptr<YoloDetector> initYoloDetector();
  // 检测装甲板 (根据 use_yolo_ 选择检测模式)
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  // 动态创建/销毁调试发布者
  void createDebugPublishers();
  void destroyDebugPublishers();

  // 发布 RViz 可视化标记
  void publishMarkers();

  // 任务模式订阅 (aim=自瞄, 其他=非自瞄任务如能量机关)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  bool is_aim_task_;

  // 检测模式: true=YOLO深度学习, false=传统CV
  bool use_yolo_;

  // 传统CV检测器
  std::unique_ptr<Detector> detector_;
  // YOLO 深度学习检测器
  std::unique_ptr<YoloDetector> yolo_detector_;

  // 检测结果发布者
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // RViz 可视化标记
  visualization_msgs::msg::Marker armor_marker_;   // 装甲板立方体标记
  visualization_msgs::msg::Marker text_marker_;    // 文字标记 (分类结果)
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 相机内参
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;                                    // 图像中心坐标
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;    // 相机信息缓存
  std::unique_ptr<PnPSolver> pnp_solver_;                     // PnP 解算器

  // 图像订阅
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Robot color subscription (for dynamic detect_color update) - disabled, use detect_color param only
  // rclcpp::Subscription<vision_interfaces::msg::Robot>::SharedPtr robot_sub_;
  // int detect_color_{0};  // 0=RED, 1=BLUE

  // 调试开关
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;   // 二值化图像
  image_transport::Publisher number_img_pub_;   // 数字识别图像
  image_transport::Publisher result_img_pub_;   // 检测结果标注图像
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
