#include "EKF.hpp"
#include <cmath>

// EKF 状态定义:
// x = [px, py, pz, vx, vy, vz, ax, ay, az]^T
// 其中 p/v/a 分别是位置、速度、加速度
EKF::EKF(double dt) : dt_(dt)
{
    // 9维状态初始化
    x_ = cv::Mat::zeros(9, 1, CV_64F);

    // 协方差初始化
    P_ = cv::Mat::eye(9, 9, CV_64F) * 100.0;

    // 过程噪声协方差 Q（越大越相信模型有随机扰动）
    Q_ = cv::Mat::eye(9, 9, CV_64F) * 0.05;

    // 像素域测量噪声 R: [u, v, 目标宽度像素值]
    R_ = cv::Mat::eye(3, 3, CV_64F);
    R_.at<double>(0, 0) = 16.0;
    R_.at<double>(1, 1) = 16.0;
    R_.at<double>(2, 2) = 64.0;

    // 控制输入加速度
    u_ = cv::Mat::zeros(3, 1, CV_64F);
}

cv::Mat EKF::stateTransition(const cv::Mat& x, const cv::Mat& u)
{
    cv::Mat x_new = x.clone();

    // 位置
    double px = x.at<double>(0);
    double py = x.at<double>(1);
    double pz = x.at<double>(2);

    // 速度
    double vx = x.at<double>(3);
    double vy = x.at<double>(4);
    double vz = x.at<double>(5);

    // 加速度
    double ax = x.at<double>(6);
    double ay = x.at<double>(7);
    double az = x.at<double>(8);

    // 控制输入
    double uax = u.at<double>(0);
    double uay = u.at<double>(1);
    double uaz = u.at<double>(2);

    // 离散常加速度模型:
    // p(k+1)=p(k)+v(k)dt+0.5*a(k)dt^2
    x_new.at<double>(0) = px + vx * dt_ + 0.5 * ax * dt_ * dt_;
    x_new.at<double>(1) = py + vy * dt_ + 0.5 * ay * dt_ * dt_;
    x_new.at<double>(2) = pz + vz * dt_ + 0.5 * az * dt_ * dt_;

    // v(k+1)=v(k)+a(k)dt
    x_new.at<double>(3) = vx + ax * dt_;
    x_new.at<double>(4) = vy + ay * dt_;
    x_new.at<double>(5) = vz + az * dt_;

    // 将控制输入作为下一时刻加速度
    x_new.at<double>(6) = uax;
    x_new.at<double>(7) = uay;
    x_new.at<double>(8) = uaz;

    return x_new;
}

cv::Mat EKF::computeJacobianF()
{
    // 状态转移雅可比 F = df/dx
    cv::Mat F = cv::Mat::eye(9, 9, CV_64F);

    // 位置对速度
    F.at<double>(0, 3) = dt_;
    F.at<double>(1, 4) = dt_;
    F.at<double>(2, 5) = dt_;

    // 位置对加速度
    F.at<double>(0, 6) = 0.5 * dt_ * dt_;
    F.at<double>(1, 7) = 0.5 * dt_ * dt_;
    F.at<double>(2, 8) = 0.5 * dt_ * dt_;

    // 速度对加速度
    F.at<double>(3, 6) = dt_;
    F.at<double>(4, 7) = dt_;
    F.at<double>(5, 8) = dt_;

    return F;
}


void EKF::predict()
{
    // 预测步骤: x(k|k) -> x(k+1|k)
    x_ = stateTransition(x_, u_);

    // 线性化后的状态转移矩阵
    cv::Mat F = computeJacobianF();

    // 协方差传播: P = FPF^T + Q
    P_ = F * P_ * F.t() + Q_;
}

cv::Mat EKF::observationModel(const cv::Mat& x) const
{
    const double px = x.at<double>(0);
    const double py = x.at<double>(1);
    const double pz = std::max(x.at<double>(2), min_depth_);

    const double u = fx_ * px / pz + cx_;
    const double v = fx_ * py / pz + cy_;
    const double w = fx_ * armor_width_ / pz;

    return (cv::Mat_<double>(3, 1) << u, v, w);
}

cv::Mat EKF::computeJacobianH(const cv::Mat& x) const
{
    const double px = x.at<double>(0);
    const double py = x.at<double>(1);
    const double pz = std::max(x.at<double>(2), min_depth_);
    const double z2 = pz * pz;

    // 观测雅可比 H = dh/dx，只和位置态 [px, py, pz] 有关
    cv::Mat H = cv::Mat::zeros(3, 9, CV_64F);
    H.at<double>(0, 0) = fx_ / pz;
    H.at<double>(0, 2) = -fx_ * px / z2;

    H.at<double>(1, 1) = fx_ / pz;
    H.at<double>(1, 2) = -fx_ * py / z2;

    H.at<double>(2, 2) = -fx_ * armor_width_ / z2;
    return H;
}

void EKF::updatePixelMeasurement(double meas_u, double meas_v, double meas_w)
{
    // 测量向量 z = [u, v, w_pixel]
    cv::Mat z = (cv::Mat_<double>(3, 1) << meas_u, meas_v, meas_w);

    // 在当前状态点线性化
    cv::Mat H = computeJacobianH(x_);
    cv::Mat z_pred = observationModel(x_);

    // 创新/残差: y = z - h(x)
    cv::Mat y = z - z_pred;

    // 残差协方差
    cv::Mat S = H * P_ * H.t() + R_;

    // 卡尔曼增益
    cv::Mat K = P_ * H.t() * S.inv();

    // 更新状态
    x_ = x_ + K * y;

    // Joseph 形式协方差更新，数值稳定性更好
    cv::Mat I = cv::Mat::eye(9, 9, CV_64F);
    const cv::Mat I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.t() + K * R_ * K.t();
}


void EKF::setControlInput(double ax, double ay, double az)
{
    // 外部估计的加速度输入
    u_.at<double>(0) = ax;
    u_.at<double>(1) = ay;
    u_.at<double>(2) = az;
}

void EKF::setCameraParams(double fx, double cx, double cy, double armor_width)
{
    fx_ = fx;
    cx_ = cx;
    cy_ = cy;
    armor_width_ = armor_width;
}


cv::Point3f EKF::getPosition() const
{
    // 返回当前滤波状态中的位置估计
    return cv::Point3f(
        (float)x_.at<double>(0),
        (float)x_.at<double>(1),
        (float)x_.at<double>(2)
    );
}
cv::Point3f EKF::getVelocity() const
{
    // 返回当前滤波状态中的速度估计
    return cv::Point3f(
        (float)x_.at<double>(3),
        (float)x_.at<double>(4),
        (float)x_.at<double>(5)
    );
}
cv::Point3f EKF::getAcceleration() const
{
    // 返回当前滤波状态中的加速度估计
    return cv::Point3f(
        (float)x_.at<double>(6),
        (float)x_.at<double>(7),
        (float)x_.at<double>(8)
    );
}
double EKF::getTimeStep() const
{
    return dt_;
}

void EKF::setInitialCovarianceScale(double covariance_scale)
{
    const double clamped_scale = std::max(covariance_scale, 1e-9);
    P_ = cv::Mat::eye(9, 9, CV_64F) * clamped_scale;
}

void EKF::setProcessNoiseScale(double process_noise_scale)
{
    const double clamped_scale = std::max(process_noise_scale, 1e-12);
    Q_ = cv::Mat::eye(9, 9, CV_64F) * clamped_scale;
}

void EKF::setMeasurementNoise(double noise_u, double noise_v, double noise_w)
{
    R_ = cv::Mat::eye(3, 3, CV_64F);
    R_.at<double>(0, 0) = std::max(noise_u, 1e-12);
    R_.at<double>(1, 1) = std::max(noise_v, 1e-12);
    R_.at<double>(2, 2) = std::max(noise_w, 1e-12);
}
