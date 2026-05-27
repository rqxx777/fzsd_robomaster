// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
// YOLO Detector for ROS2 - Adapted from sp_vision_25

#ifndef ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
#define ARMOR_DETECTOR__YOLO_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{

// YOLO目标检测的五元组输出
struct YoloArmor
{
  int class_id;                             // 类别ID (0~37)
  float confidence;                         // 检测置信度
  cv::Rect box;                             // 边界框
  std::vector<cv::Point2f> keypoints;       // 4个装甲板角点: 左上、右上、右下、左下

  // 解析后的语义属性
  int color;              // 0=blue, 1=red
  std::string number;     // 数字标签 ("1"~"5"/"sentry"/"outpost"/"base")
  ArmorType type;         // 装甲板大小类型
};

// 基于 YOLO + OpenVINO 的深度学习装甲板检测器
class YoloDetector
{
public:
  struct YoloParams
  {
    std::string model_path;           // OpenVINO 模型路径 (.xml)
    std::string device = "CPU";       // 推理设备: CPU/GPU
    int input_size = 640;             // YOLO11=640, YOLOv8=416
    int class_num = 38;               // 类别数 (4种颜色 × 多类数字 + 大/小装甲板)
    float score_threshold = 0.7;      // 置信度阈值，低分检测框会被过滤
    float nms_threshold = 0.3;        // NMS IoU 阈值，值越大保留越多重叠框
    float min_confidence = 0.8;       // 最终有效检测的最低置信度
    bool use_roi = false;             // 是否启用 ROI 区域裁剪
    cv::Rect roi;                     // ROI 区域
  };

  explicit YoloDetector(const YoloParams & params);

  // 检测装甲板，返回装甲板列表 (与传统检测器接口兼容)
  std::vector<Armor> detect(const cv::Mat & img);

  // 调试用: 在原图上绘制检测结果
  void drawResults(cv::Mat & img, const std::vector<Armor> & armors);

private:
  YoloParams params_;

  ov::Core core_;                         // OpenVINO 核心对象
  ov::CompiledModel compiled_model_;      // 编译后的模型

  cv::Point2f offset_;                    // ROI 偏移量

  // 38类装甲板语义属性映射表: {颜色, 数字标签, 装甲板大小类型}
  static const std::vector<std::tuple<int, std::string, ArmorType>> armor_classes_;

  // 图像预处理: letterbox 缩放至模型输入尺寸
  // scale: 原始图像到输入尺寸的缩放比例
  cv::Mat preprocess(const cv::Mat & img, double & scale);

  // YOLO输出后处理: 解析边界框+关键点，NMS去重
  std::vector<YoloArmor> postprocess(double scale, const ov::Tensor & output_tensor);

  // 将 YoloArmor 转换为系统内部的 Armor 结构体
  Armor convertToArmor(const YoloArmor & yolo_armor);

  // 对4个角点排序: 左上(0)、右上(1)、右下(2)、左下(3)
  void sortKeypoints(std::vector<cv::Point2f> & keypoints);

  // 验证装甲板有效性 (置信度阈值 + 数字非空检查)
  bool checkArmor(const YoloArmor & armor);

  // 从 class_id 解析颜色、数字标签、装甲板大小类型
  void parseClassId(int class_id, int & color, std::string & number, ArmorType & type);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
