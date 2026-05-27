// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
// 传统CV检测器: 通过二值化→轮廓提取→灯条匹配来检测装甲板
class Detector
{
public:
  // 灯条筛选参数
  struct LightParams
  {
    double min_ratio;       // 最小宽高比 (宽度/长度)
    double max_ratio;       // 最大宽高比
    double max_angle;       // 最大倾斜角度 (度)
    double min_fill_ratio;  // 最小填充率 (轮廓面积/外接旋转矩形面积)
  };

  // 装甲板匹配参数
  struct ArmorParams
  {
    double min_light_ratio;              // 两灯条最小长度比
    double min_small_center_distance;    // 小装甲板最小灯条间距 (单位为灯条长度)
    double max_small_center_distance;    // 小装甲板最大灯条间距
    double min_large_center_distance;    // 大装甲板最小灯条间距
    double max_large_center_distance;    // 大装甲板最大灯条间距
    double max_angle;                    // 灯条连线最大水平夹角 (度)
  };

  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  // 主检测流程: 预处理 → 找灯条 → 匹配装甲板 → 数字识别
  std::vector<Armor> detect(const cv::Mat & input);

  // 图像预处理: 灰度化 + 二值化
  cv::Mat preprocessImage(const cv::Mat & input);
  // 轮廓提取与灯条筛选
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  // 灯条配对匹配装甲板
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // 调试用: 获取所有识别数字的拼接图像
  cv::Mat getAllNumbersImage();
  // 调试用: 在原图上绘制检测结果
  void drawResults(cv::Mat & img);

  int binary_thres;     // 二值化阈值
  int detect_color;     // 目标颜色: RED(0) 或 BLUE(1)
  LightParams l;        // 灯条参数
  ArmorParams a;        // 装甲板参数

  std::unique_ptr<NumberClassifier> classifier;  // 数字分类器

  // 调试数据
  cv::Mat binary_img;                              // 二值化结果图像
  auto_aim_interfaces::msg::DebugLights debug_lights;   // 灯条调试信息
  auto_aim_interfaces::msg::DebugArmors debug_armors;   // 装甲板调试信息

private:
  // 验证单个灯条是否满足筛选条件
  bool isLight(const Light & possible_light);
  // 检查两灯条之间是否包含第三个灯条 (排除嵌套情况)
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  // 验证一对灯条是否构成有效装甲板，返回装甲板类型
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
