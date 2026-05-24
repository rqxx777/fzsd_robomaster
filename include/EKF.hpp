#pragma once
#include <opencv2/opencv.hpp>

class EKF
{
public:
    EKF(double dt = 0.033);

    // 预测
    void predict();

    // 更新
    void updateXYZ(double meas_x, double meas_y, double meas_z);

    // 获取预测位置
    cv::Point3f getPosition() const;

    // 设置控制输入
    void setControlInput(double ax, double ay, double az);

private:
    double dt_;

    // 状态向量 x = [x,y,z,vx,vy,vz,ax,ay,az]^T
    cv::Mat x_;

    // 协方差矩阵
    cv::Mat P_;

    // 过程噪声
    cv::Mat Q_;

    // 测量噪声
    cv::Mat R_;

    // 控制输入
    cv::Mat u_;

    // 状态转移函数
    cv::Mat stateTransition(const cv::Mat& x, const cv::Mat& u);

    // 雅可比矩阵
    cv::Mat computeJacobianF();
};
