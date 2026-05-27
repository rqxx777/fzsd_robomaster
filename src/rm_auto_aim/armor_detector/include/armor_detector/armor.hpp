// Copyright 2022 Chen Jun
// Copyright 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{
// 目标颜色常量: RED=攻击红色方, BLUE=攻击蓝色方
const int RED = 0;
const int BLUE = 1;

// 装甲板类型: SMALL=小装甲板(132x57mm), LARGE=大装甲板(223x57mm)
enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

// 灯条结构体: 继承自 cv::Rect 表示外接矩形，额外包含顶点和颜色信息
struct Light : public cv::Rect
{
  Light() = default;
  explicit Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
  : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle)
  {
    length = cv::norm(top - bottom);  // 灯条长度 (像素)
    width = area / length;            // 灯条宽度 = 面积 / 长度
    center = (top + bottom) / 2;      // 灯条中心点
  }

  int color;                // 灯条颜色: RED(0) 或 BLUE(1)
  cv::Point2f top, bottom;  // 灯条两端端点坐标
  cv::Point2f center;       // 灯条中心点坐标
  double length;            // 灯条长度 (像素)
  double width;             // 灯条宽度 (像素)
  float tilt_angle;         // 灯条倾斜角度 (度)
};

// 装甲板结构体: 由一对匹配的灯条构成，包含数字识别结果
struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    // 按 x 坐标确定左右灯条
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // 灯条对
  Light left_light, right_light;
  cv::Point2f center;         // 装甲板中心点 (图像坐标)
  ArmorType type;             // 装甲板类型 (大/小)

  // 数字识别结果
  cv::Mat number_img;         // 提取的数字ROI图像
  std::string number;         // 识别出的数字标签 ("1"~"5"/"sentry"/"outpost"/"base")
  float confidence;           // 分类置信度
  std::string classfication_result;  // 最终分类结果字符串
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
