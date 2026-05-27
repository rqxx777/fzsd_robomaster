// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
// 装甲板数字分类器: 使用 MLP (多层感知机) 模型识别装甲板上的数字
class NumberClassifier
{
public:
  // model_path: ONNX 模型路径, label_path: 标签文件路径
  // threshold: 分类置信度阈值, ignore_classes: 忽略的类别 (如 "negative")
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  // 从原图中提取装甲板区域的数字 ROI 图像
  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  // 对提取的数字 ROI 进行分类识别
  void classify(std::vector<Armor> & armors);

  double threshold;  // 分类置信度阈值

private:
  cv::dnn::Net net_;                          // OpenCV DNN 网络
  std::vector<std::string> class_names_;       // 类别名称列表
  std::vector<std::string> ignore_classes_;    // 忽略的类别
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
