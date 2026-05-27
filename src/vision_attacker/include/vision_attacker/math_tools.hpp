#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <chrono>

namespace tools
{
// 将弧度值归一化到 (-π, π] 范围
double limit_rad(double angle);

// 四元数/旋转矩阵 → 欧拉角 (支持任意旋转顺序)
// axis 参数: x=0, y=1, z=2
// extrinsic=true: 外旋 (绕固定轴), false: 内旋 (绕自身轴)
// 例: 先绕Z轴→再绕Y轴→最后绕X轴: axis0=2, axis1=1, axis2=0
// 参考: https://github.com/evbernardes/quaternion_to_euler
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// ZYX 欧拉角 → 旋转矩阵 (先绕Z轴→再绕Y轴→最后绕X轴)
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

// 直角坐标 (x, y, z) → 球坐标 (yaw, pitch, distance)
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// xyz→ypd 转换的雅可比矩阵 (3×3)
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 球坐标 (yaw, pitch, distance) → 直角坐标 (x, y, z)
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

// ypd→xyz 转换的雅可比矩阵 (3×3)
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

// 计算时间点 a - b 的时间差 (秒)
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

// 计算两个二维向量之间的夹角，返回值范围 [0, π]
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2);

// 返回输入值的平方
template <typename T>
T square(T const & a)
{
  return a * a;
};

// 将输入值限制在 [min, max] 范围内
double limit_min_max(double input, double min, double max);
}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP