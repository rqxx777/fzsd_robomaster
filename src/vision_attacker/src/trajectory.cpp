#include "vision_attacker/trajectory.hpp"
#include <cmath>

  // 一阶空气阻力弹道模型: a_drag = -air_k * |v| * v, 重力 g = 9.78 m/s²
double solveTrajectory(double &flyTime, const double x, const double z, const double lagTime, const double muzzleSpeed, const double air_k)
  // 迭代20次修正发射角度: 用当前角模拟弹道 → 计算高度误差 → 修正角度
{
    const double gravity = 9.78;
    double dt = 0.002; // 步长
    double px = 0.0, pz = 0.0; // 起点
    double zFix = 0.0;
    double xFix = 0.0;
    if (x > 3)
    {
        zFix = (x-3)*0.07+0.05; // 根据距离动态调整z修正
        // 暂时不需要保护逻辑，先注释
        // if (zFix+z>-0.03)
        // {
        //     zFix = -0.03-z;
        //     if (z > -0.02)
        //     {
        //         zFix = 0.0;
        //     }
        // }
    }
    // if (x > 3)
    // {
    //     xFix = (x-3)*(x-3) * 0.5+0.05; // 根据距离动态调整x修正
    // }
    double vx = muzzleSpeed, vz = 0.0;
    double theta = atan2(z, x);
    vx = muzzleSpeed * cos(theta);
    vz = muzzleSpeed * sin(theta);
    double t = 0.0;
    double px_target = x+xFix;
    double pz_target = z+zFix ;
    for (size_t i = 0; i < 20; i++) {
        px = 0.0; pz = 0.0;
        vx = muzzleSpeed * cos(theta);
        vz = muzzleSpeed * sin(theta);
        t = 0.0;
         int max_steps = 10000; // 防止死循环
        while (px < px_target && vx > 0.01 && max_steps-- > 0) {
            double v = sqrt(vx*vx + vz*vz);
            vx -= air_k * v * vx * dt;
            vz -= gravity * dt + air_k * v * vz * dt;
            px += vx * dt;
            pz += vz * dt;
            t += dt;
        }
        double dz = pz_target - pz;
        if (std::abs(dz) < 0.001) {
            flyTime = t + lagTime;
            return atan2(pz, px_target);
        }
        theta += dz / px_target; // 修正角度
    }
    flyTime = t + lagTime;
    return atan2(pz, px_target);
}
