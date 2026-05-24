#pragma once
#include <opencv2/opencv.hpp>

class EKF
{
public:
    EKF(double dt = 0.033);

    // 预测
    void predict();

    // 非线性像素观测更新: z = [u, v, w_pixel]^T
    void updatePixelMeasurement(double meas_u, double meas_v, double meas_w);

    // 设置相机与目标先验参数
    void setCameraParams(double fx, double cx, double cy, double armor_width);

    // 获取预测位置
    cv::Point3f getPosition() const;

    // 设置控制输入
    void setControlInput(double ax, double ay, double az);

    cv::Point3f getVelocity() const;
    cv::Point3f getAcceleration() const;
    double getTimeStep() const;
    void setInitialCovarianceScale(double covariance_scale);
    void setProcessNoiseScale(double process_noise_scale);
    void setMeasurementNoise(double noise_u, double noise_v, double noise_w);


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

    // 观测函数 h(x) = [fx*x/z + cx, fx*y/z + cy, fx*W/z]^T
    cv::Mat observationModel(const cv::Mat& x) const;

    // 观测雅可比 H = dh/dx
    cv::Mat computeJacobianH(const cv::Mat& x) const;

    double fx_ = 1000.0;
    double cx_ = 640.0;
    double cy_ = 360.0;
    double armor_width_ = 0.135;
    double min_depth_ = 1e-3;
};
