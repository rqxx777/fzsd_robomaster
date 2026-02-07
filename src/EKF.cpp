#include "EKF.hpp"
#include <cmath>

EKF::EKF(double dt) : dt_(dt)
{
    // 9维状态初始化
    x_ = cv::Mat::zeros(9, 1, CV_64F);

    // 协方差初始化
    P_ = cv::Mat::eye(9, 9, CV_64F) * 100.0;

    // 过程噪声
    Q_ = cv::Mat::eye(9, 9, CV_64F) * 0.05;

    // 测量噪声
    R_ = cv::Mat::eye(3, 3, CV_64F) * 5.0;

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

    // 更新位置
    x_new.at<double>(0) = px + vx * dt_ + 0.5 * ax * dt_ * dt_;
    x_new.at<double>(1) = py + vy * dt_ + 0.5 * ay * dt_ * dt_;
    x_new.at<double>(2) = pz + vz * dt_ + 0.5 * az * dt_ * dt_;

    // 更新速度
    x_new.at<double>(3) = vx + ax * dt_;
    x_new.at<double>(4) = vy + ay * dt_;
    x_new.at<double>(5) = vz + az * dt_;

    // 更新加速度
    x_new.at<double>(6) = ax + uax;
    x_new.at<double>(7) = ay + uay;
    x_new.at<double>(8) = az + uaz;

    return x_new;
}

cv::Mat EKF::computeJacobianF()
{
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
    // 状态预测
    x_ = stateTransition(x_, u_);

    // 雅可比矩阵
    cv::Mat F = computeJacobianF();

    // 协方差预测
    P_ = F * P_ * F.t() + Q_;
}

void EKF::updateXYZ(double meas_x, double meas_y, double meas_z)
{
    // 测量向量 z = [x,y,z]
    cv::Mat z = (cv::Mat_<double>(3, 1) << meas_x, meas_y, meas_z);

    // H矩阵
    cv::Mat H = cv::Mat::zeros(3, 9, CV_64F);
    H.at<double>(0, 0) = 1;
    H.at<double>(1, 1) = 1;
    H.at<double>(2, 2) = 1;

    // 残差
    cv::Mat y = z - H * x_;

    // 残差协方差
    cv::Mat S = H * P_ * H.t() + R_;

    // 卡尔曼增益
    cv::Mat K = P_ * H.t() * S.inv();

    // 更新状态
    x_ = x_ + K * y;

    // 更新协方差
    cv::Mat I = cv::Mat::eye(9, 9, CV_64F);
    P_ = (I - K * H) * P_;
}


void EKF::setControlInput(double ax, double ay, double az)
{
    u_.at<double>(0) = ax;
    u_.at<double>(1) = ay;
    u_.at<double>(2) = az;
}


cv::Point3f EKF::getPosition() const
{
    return cv::Point3f(
        (float)x_.at<double>(0),
        (float)x_.at<double>(1),
        (float)x_.at<double>(2)
    );
}
