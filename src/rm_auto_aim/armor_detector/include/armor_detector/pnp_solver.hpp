// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
// PnP 姿态解算器: 根据装甲板四角图像坐标和实际物理尺寸，解算3D位姿
class PnPSolver
{
public:
  // camera_matrix: 3x3 相机内参矩阵 (fx, fy, cx, cy)
  // distortion_coefficients: 5 参数畸变系数 (k1,k2,p1,p2,k3)
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // 使用 IPPE 算法解算装甲板的旋转向量 rvec 和平移向量 tvec
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // 计算装甲板中心到图像中心的距离 (用于选择最接近画面中心的装甲板)
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;  // 相机内参矩阵
  cv::Mat dist_coeffs_;    // 畸变系数

  // 装甲板实际物理尺寸 (单位: mm)
  static constexpr float SMALL_ARMOR_WIDTH = 132;
  static constexpr float SMALL_ARMOR_HEIGHT = 57;
  static constexpr float LARGE_ARMOR_WIDTH = 223;
  static constexpr float LARGE_ARMOR_HEIGHT = 57;

  // 装甲板四角在模型坐标系下的3D坐标 (x=0 平面, y 为宽度方向, z 为高度方向)
  // 顺序: 左下、左上、右上、右下 (顺时针)
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
