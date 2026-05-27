#pragma once
#include <chrono>

// 一阶空气阻力弹道解算
// 输入端: 目标水平距离 x, 目标高度 z, 延迟时间 lagTime, 弹丸初速 muzzleSpeed, 空气阻力系数 air_k
// 输出端: flyTime (飞行时间+延迟), 返回值 = 发射Pitch角 (rad)
// 模型: a_drag = -air_k * |v| * v, 重力加速度 g = 9.78 m/s²
// 算法: 迭代修正发射角直到弹道命中目标位置
double solveTrajectory(double &flyTime, const double x, const double z, const double lagTime, const double muzzleSpeed, const double air_k);
