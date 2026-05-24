#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "armor_detect_ekf/msg/target_state.hpp"
#include "armor_detect.hpp"
#include "EKF.hpp"

using std::placeholders::_1;

namespace
{

cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    try {
        return cv_bridge::toCvShare(img_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("detect_ekf_node"), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

bool solveArmorPnPFromBox(
    const Detection & det,
    double fx, double fy, double cx, double cy,
    double armor_w, double armor_h,
    cv::Point3d & pos_cam)
{
    if (det.box.width <= 1 || det.box.height <= 1) {
        return false;
    }

    const std::vector<cv::Point3f> object_points{
        cv::Point3f(static_cast<float>(-armor_w * 0.5), static_cast<float>(-armor_h * 0.5), 0.0f),
        cv::Point3f(static_cast<float>( armor_w * 0.5), static_cast<float>(-armor_h * 0.5), 0.0f),
        cv::Point3f(static_cast<float>( armor_w * 0.5), static_cast<float>( armor_h * 0.5), 0.0f),
        cv::Point3f(static_cast<float>(-armor_w * 0.5), static_cast<float>( armor_h * 0.5), 0.0f)
    };

    const int x = det.box.x;
    const int y = det.box.y;
    const int w = det.box.width;
    const int h = det.box.height;
    const std::vector<cv::Point2f> image_points{
        cv::Point2f(static_cast<float>(x), static_cast<float>(y)),
        cv::Point2f(static_cast<float>(x + w), static_cast<float>(y)),
        cv::Point2f(static_cast<float>(x + w), static_cast<float>(y + h)),
        cv::Point2f(static_cast<float>(x), static_cast<float>(y + h))
    };

    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    const cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    cv::Mat rvec;
    cv::Mat tvec;
    const bool ok = cv::solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    if (!ok || tvec.rows != 3) {
        return false;
    }

    pos_cam.x = tvec.at<double>(0);
    pos_cam.y = tvec.at<double>(1);
    pos_cam.z = tvec.at<double>(2);

    return std::isfinite(pos_cam.x) && std::isfinite(pos_cam.y) && std::isfinite(pos_cam.z) && pos_cam.z > 0.0;
}

}  // namespace

class ArmorDetectEkfNode : public rclcpp::Node
{
private:
    struct MeasurementResult
    {
        bool valid{false};
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double u{0.0};
        double v{0.0};
        double w_pixel{0.0};
    };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pub_;
    rclcpp::Publisher<armor_detect_ekf::msg::TargetState>::SharedPtr target_state_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    EKF ekf_{0.033};
    ArmorDetector detector_;

    const double fx_ = 1000.0;
    const double fy_ = 1000.0;
    const double cx_ = 640.0;
    const double cy_ = 360.0;
    const double armor_w_ = 0.135;
    const double armor_h_ = 0.055;

    double marker_velocity_scale_ = 0.20;

    // 加速度限幅和低通滤波，防止预测震荡
    double max_acceleration_ = 50.0;
    double accel_lowpass_alpha_ = 0.3;
    cv::Point3f filtered_acceleration_{0.0f, 0.0f, 0.0f};

    bool enable_tuning_gui_ = true;
    bool gui_initialized_ = false;
    std::string gui_window_name_ = "ekf_tuning";
    bool show_detection_image_ = false;
    std::string detection_window_name_ = "armor_detection";
    bool draw_prediction_on_image_ = true;
    bool draw_ballistic_on_image_ = true;
    double ballistic_bullet_speed_ = 20.0;
    double ballistic_gravity_ = 9.81;
    double ballistic_delay_ = 0.1;

    int tb_p0_ = 100;
    int tb_q_ = 50;
    int tb_ru_ = 16;
    int tb_rv_ = 16;
    int tb_rw_ = 64;
    int tb_viz_vel_ = 20;
    int tb_max_accel_ = 50;
    int tb_accel_alpha_ = 30;

    static geometry_msgs::msg::Point toGeometryPoint(double x, double y, double z)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    MeasurementResult estimateFromDetection(const Detection & d) const
    {
        MeasurementResult meas;
        meas.u = d.box.x + d.box.width / 2.0;
        meas.v = d.box.y + d.box.height / 2.0;
        meas.w_pixel = d.box.width;

        cv::Point3d pnp_pos;
        if (solveArmorPnPFromBox(d, fx_, fy_, cx_, cy_, armor_w_, armor_h_, pnp_pos)) {
            meas.x = pnp_pos.x;
            meas.y = pnp_pos.y;
            meas.z = pnp_pos.z;
            meas.valid = true;
            return meas;
        }

        const double depth = armor_w_ * fx_ / std::max(1.0, meas.w_pixel);
        meas.x = (meas.u - cx_) * depth / fx_;
        meas.y = (meas.v - cy_) * depth / fy_;
        meas.z = depth;
        meas.valid = true;
        return meas;
    }

    void initTuningGui()
    {
        if (!enable_tuning_gui_ || gui_initialized_) {
            return;
        }

        cv::namedWindow(gui_window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(gui_window_name_, 1200, 800);
        cv::createTrackbar("P0 x1", gui_window_name_, &tb_p0_, 1000);
        cv::createTrackbar("Q x1000", gui_window_name_, &tb_q_, 1000);
        cv::createTrackbar("Ru", gui_window_name_, &tb_ru_, 500);
        cv::createTrackbar("Rv", gui_window_name_, &tb_rv_, 500);
        cv::createTrackbar("Rw", gui_window_name_, &tb_rw_, 1000);
        cv::createTrackbar("VelViz x100", gui_window_name_, &tb_viz_vel_, 200);
        cv::createTrackbar("MaxAccel", gui_window_name_, &tb_max_accel_, 200);
        cv::createTrackbar("AccelAlpha x100", gui_window_name_, &tb_accel_alpha_, 100);
        gui_initialized_ = true;
    }

    void applyTuningFromGui()
    {
        if (!enable_tuning_gui_ || !gui_initialized_) {
            return;
        }

        const double p0 = std::max(tb_p0_ * 1.0, 1e-9);
        const double q = std::max(tb_q_ / 1000.0, 1e-12);
        const double ru = std::max(tb_ru_ * 1.0, 1e-12);
        const double rv = std::max(tb_rv_ * 1.0, 1e-12);
        const double rw = std::max(tb_rw_ * 1.0, 1e-12);

        marker_velocity_scale_ = std::max(tb_viz_vel_ / 100.0, 0.0);

        max_acceleration_ = std::max(tb_max_accel_ * 1.0, 1.0);
        accel_lowpass_alpha_ = std::clamp(tb_accel_alpha_ / 100.0, 0.01, 1.0);

        ekf_.setInitialCovarianceScale(p0);
        ekf_.setProcessNoiseScale(q);
        ekf_.setMeasurementNoise(ru, rv, rw);
    }

    void showTuningView(const cv::Mat & image, const cv::Point3f & pred, const cv::Point3f & vel)
    {
        if (!enable_tuning_gui_ || !gui_initialized_) {
            return;
        }

        cv::Mat vis = image.clone();
        const cv::Scalar text_color(0, 255, 255);

        cv::putText(vis, "EKF GUI Tuning", {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);

        char line[256];
        std::snprintf(
            line,
            sizeof(line),
            "P0=%.2f Q=%.4f Ru=%.1f Rv=%.1f Rw=%.1f VelScale=%.2f MaxAccel=%.0f AccelAlpha=%.2f",
            std::max(tb_p0_ * 1.0, 1e-9),
            std::max(tb_q_ / 1000.0, 1e-12),
            std::max(tb_ru_ * 1.0, 1e-12),
            std::max(tb_rv_ * 1.0, 1e-12),
            std::max(tb_rw_ * 1.0, 1e-12),
            marker_velocity_scale_,
            max_acceleration_,
            accel_lowpass_alpha_);
        cv::putText(vis, line, {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);

        std::snprintf(
            line,
            sizeof(line),
            "Pred:(%.2f, %.2f, %.2f) Vel:(%.2f, %.2f, %.2f)",
            pred.x,
            pred.y,
            pred.z,
            vel.x,
            vel.y,
            vel.z);
        cv::putText(vis, line, {20, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);

        cv::imshow(gui_window_name_, vis);
        cv::waitKey(1);
    }

    cv::Point2f projectToPixel(const cv::Point3f & p_cam) const
    {
        const double z = std::max(static_cast<double>(p_cam.z), 1e-6);
        const float u = static_cast<float>(fx_ * p_cam.x / z + cx_);
        const float v = static_cast<float>(fy_ * p_cam.y / z + cy_);
        return cv::Point2f(u, v);
    }

    double ballisticResidual(
        double t,
        const cv::Point3f & position,
        const cv::Point3f & velocity,
        const cv::Point3f & acceleration) const
    {
        // 坐标系约定：camera_frame, Z朝前(深度), X朝右, Y朝下
        // 子弹受重力下坠 -> Y轴正方向
        const double predict_t = t + ballistic_delay_;
        const double x_t = position.x + velocity.x * predict_t + 0.5 * acceleration.x * predict_t * predict_t;
        const double y_t = position.y + velocity.y * predict_t + 0.5 * acceleration.y * predict_t * predict_t;
        const double z_t = position.z + velocity.z * predict_t + 0.5 * acceleration.z * predict_t * predict_t;
        const double y_with_gravity = y_t + 0.5 * ballistic_gravity_ * t * t;
        return x_t * x_t + y_with_gravity * y_with_gravity + z_t * z_t -
               ballistic_bullet_speed_ * ballistic_bullet_speed_ * t * t;
    }

    std::optional<cv::Point3f> solveBallisticAimPoint(
        const cv::Point3f & position,
        const cv::Point3f & velocity,
        const cv::Point3f & acceleration) const
    {
        constexpr double kTimeMin = 1e-3;
        constexpr double kTimeMax = 3.0;
        constexpr double kScanStep = 2e-3;
        constexpr int kBisectionIters = 60;

        double left = kTimeMin;
        double f_left = ballisticResidual(left, position, velocity, acceleration);
        bool bracketed = false;
        double right = left;
        double f_right = f_left;

        for (double t = kTimeMin + kScanStep; t <= kTimeMax; t += kScanStep) {
            const double f_t = ballisticResidual(t, position, velocity, acceleration);
            if (std::abs(f_left) < 1e-8) {
                right = left;
                f_right = f_left;
                bracketed = true;
                break;
            }
            if (f_left * f_t <= 0.0) {
                right = t;
                f_right = f_t;
                bracketed = true;
                break;
            }
            left = t;
            f_left = f_t;
        }

        if (!bracketed) {
            return std::nullopt;
        }

        double t_hit = right;
        if (std::abs(f_right) >= 1e-8) {
            double l = left;
            double r = right;
            double fl = f_left;
            for (int i = 0; i < kBisectionIters; ++i) {
                const double mid = 0.5 * (l + r);
                const double fm = ballisticResidual(mid, position, velocity, acceleration);
                if (fl * fm <= 0.0) {
                    r = mid;
                } else {
                    l = mid;
                    fl = fm;
                }
            }
            t_hit = 0.5 * (l + r);
        }

        const double predict_t = t_hit + ballistic_delay_;
        const cv::Point3f aim_point{
            static_cast<float>(position.x + velocity.x * predict_t + 0.5 * acceleration.x * predict_t * predict_t),
            static_cast<float>(position.y + velocity.y * predict_t + 0.5 * acceleration.y * predict_t * predict_t),
            static_cast<float>(position.z + velocity.z * predict_t + 0.5 * acceleration.z * predict_t * predict_t)
        };
        return aim_point;
    }

    void showDetectionImage(
        const cv::Mat & image,
        const std::vector<Detection> & detections,
        const cv::Point3f & prediction,
        const cv::Point3f & velocity,
        const cv::Point3f & acceleration)
    {
        if (!show_detection_image_) {
            return;
        }

        cv::Mat vis = image.clone();
        for (const auto & d : detections) {
            cv::rectangle(vis, d.box, cv::Scalar(0, 255, 0), 2);
            char label_buf[64];
            std::snprintf(label_buf, sizeof(label_buf), "id:%d conf:%.2f", d.class_id, d.conf);
            const std::string label(label_buf);
            cv::putText(
                vis,
                label,
                cv::Point(d.box.x, std::max(20, d.box.y - 6)),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 255),
                1);
        }

        if (draw_prediction_on_image_ && prediction.z > 1e-6f) {
            const cv::Point2f pred_uv = projectToPixel(prediction);
            cv::drawMarker(
                vis,
                pred_uv,
                cv::Scalar(255, 0, 0),
                cv::MARKER_CROSS,
                20,
                2);
            cv::putText(
                vis,
                "EKF Pred",
                cv::Point(static_cast<int>(pred_uv.x + 6), static_cast<int>(pred_uv.y - 6)),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(255, 0, 0),
                1);
        }

        if (draw_ballistic_on_image_) {
            const auto aim_point = solveBallisticAimPoint(prediction, velocity, acceleration);
            if (aim_point.has_value() && aim_point->z > 1e-6f) {
                const cv::Point2f aim_uv = projectToPixel(*aim_point);
                cv::circle(vis, aim_uv, 8, cv::Scalar(0, 165, 255), 2);
                cv::putText(
                    vis,
                    "Ballistic Aim",
                    cv::Point(static_cast<int>(aim_uv.x + 8), static_cast<int>(aim_uv.y + 16)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 165, 255),
                    1);
            }
        }

        cv::imshow(detection_window_name_, vis);
        cv::waitKey(1);
    }

    void updateEkfByDetection(const std::vector<Detection> & detections, MeasurementResult & measurement)
    {
        ekf_.predict();

        if (detections.empty()) {
            measurement.valid = false;
            return;
        }

        measurement = estimateFromDetection(detections.front());
        if (!measurement.valid) {
            return;
        }

        const cv::Point3f pos = ekf_.getPosition();
        const cv::Point3f vel = ekf_.getVelocity();
        const double dt = std::max(ekf_.getTimeStep(), 1e-6);

        // 从测量反算加速度
        double ax_raw = (measurement.x - pos.x - vel.x * dt) / (dt * dt);
        double ay_raw = (measurement.y - pos.y - vel.y * dt) / (dt * dt);
        double az_raw = (measurement.z - pos.z - vel.z * dt) / (dt * dt);

        // 加速度限幅，防止剧烈跳动
        ax_raw = std::clamp(ax_raw, -max_acceleration_, max_acceleration_);
        ay_raw = std::clamp(ay_raw, -max_acceleration_, max_acceleration_);
        az_raw = std::clamp(az_raw, -max_acceleration_, max_acceleration_);

        // 一阶低通滤波，平滑加速度估计
        const double alpha = accel_lowpass_alpha_;
        filtered_acceleration_.x = static_cast<float>(
            alpha * ax_raw + (1.0 - alpha) * filtered_acceleration_.x);
        filtered_acceleration_.y = static_cast<float>(
            alpha * ay_raw + (1.0 - alpha) * filtered_acceleration_.y);
        filtered_acceleration_.z = static_cast<float>(
            alpha * az_raw + (1.0 - alpha) * filtered_acceleration_.z);

        ekf_.setControlInput(
            filtered_acceleration_.x,
            filtered_acceleration_.y,
            filtered_acceleration_.z);
        ekf_.updatePixelMeasurement(measurement.u, measurement.v, measurement.w_pixel);
    }

    void appendObservationMarkers(
        const std::vector<Detection> & detections,
        const rclcpp::Time & stamp,
        visualization_msgs::msg::MarkerArray & markers) const
    {
        int id = 0;
        for (const auto & d : detections) {
            const MeasurementResult meas = estimateFromDetection(d);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_frame";
            marker.header.stamp = stamp;
            marker.ns = "armor_observation";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = toGeometryPoint(meas.x, meas.y, meas.z);
            marker.pose.orientation.w = 1.0;
            marker.scale.x = armor_w_;
            marker.scale.y = armor_h_;
            marker.scale.z = 0.02;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.65;
            marker.lifetime = rclcpp::Duration::from_seconds(0.10);
            markers.markers.push_back(marker);
        }
    }

    void appendPredictionMarkers(
        const cv::Point3f & pred,
        const cv::Point3f & velocity,
        const rclcpp::Time & stamp,
        visualization_msgs::msg::MarkerArray & markers) const
    {
        visualization_msgs::msg::Marker pred_marker;
        pred_marker.header.frame_id = "camera_frame";
        pred_marker.header.stamp = stamp;
        pred_marker.ns = "ekf_prediction";
        pred_marker.id = 1000;
        pred_marker.type = visualization_msgs::msg::Marker::CUBE;
        pred_marker.action = visualization_msgs::msg::Marker::ADD;
        pred_marker.pose.position = toGeometryPoint(pred.x, pred.y, pred.z);
        pred_marker.pose.orientation.w = 1.0;
        pred_marker.scale.x = armor_w_;
        pred_marker.scale.y = armor_h_;
        pred_marker.scale.z = 0.02;
        pred_marker.color.r = 0.1;
        pred_marker.color.g = 0.2;
        pred_marker.color.b = 1.0;
        pred_marker.color.a = 0.75;
        pred_marker.lifetime = rclcpp::Duration::from_seconds(0.10);
        markers.markers.push_back(pred_marker);

        visualization_msgs::msg::Marker vel_marker;
        vel_marker.header.frame_id = "camera_frame";
        vel_marker.header.stamp = stamp;
        vel_marker.ns = "ekf_velocity";
        vel_marker.id = 1001;
        vel_marker.type = visualization_msgs::msg::Marker::ARROW;
        vel_marker.action = visualization_msgs::msg::Marker::ADD;

        vel_marker.points.push_back(toGeometryPoint(pred.x, pred.y, pred.z));
        vel_marker.points.push_back(
            toGeometryPoint(
                pred.x + velocity.x * marker_velocity_scale_,
                pred.y + velocity.y * marker_velocity_scale_,
                pred.z + velocity.z * marker_velocity_scale_));

        vel_marker.scale.x = 0.01;
        vel_marker.scale.y = 0.02;
        vel_marker.scale.z = 0.04;
        vel_marker.color.r = 1.0;
        vel_marker.color.g = 1.0;
        vel_marker.color.b = 0.0;
        vel_marker.color.a = 0.9;
        vel_marker.lifetime = rclcpp::Duration::from_seconds(0.10);
        markers.markers.push_back(vel_marker);
    }

    void publishTargetState(
        const std_msgs::msg::Header & header,
        const cv::Point3f & pred,
        const cv::Point3f & velocity,
        const cv::Point3f & acceleration,
        const MeasurementResult & measurement)
    {
        armor_detect_ekf::msg::TargetState msg;
        msg.header = header;

        msg.position.x = pred.x;
        msg.position.y = pred.y;
        msg.position.z = pred.z;

        msg.velocity.x = velocity.x;
        msg.velocity.y = velocity.y;
        msg.velocity.z = velocity.z;

        msg.acceleration.x = acceleration.x;
        msg.acceleration.y = acceleration.y;
        msg.acceleration.z = acceleration.z;

        msg.measurement_position.x = measurement.x;
        msg.measurement_position.y = measurement.y;
        msg.measurement_position.z = measurement.z;
        msg.measurement_valid = measurement.valid;
        msg.dt = ekf_.getTimeStep();

        target_state_pub_->publish(msg);
    }

    rcl_interfaces::msg::SetParametersResult onParameterUpdate(
        const std::vector<rclcpp::Parameter> & params)
    {
        double ru = this->get_parameter("ekf.measurement_noise_u").as_double();
        double rv = this->get_parameter("ekf.measurement_noise_v").as_double();
        double rw = this->get_parameter("ekf.measurement_noise_w").as_double();
        bool measurement_noise_updated = false;

        for (const auto & param : params) {
            const std::string & name = param.get_name();
            if (name == "ekf.initial_covariance_scale") {
                ekf_.setInitialCovarianceScale(param.as_double());
            } else if (name == "ekf.process_noise_scale") {
                ekf_.setProcessNoiseScale(param.as_double());
            } else if (name == "ekf.measurement_noise_u") {
                ru = param.as_double();
                measurement_noise_updated = true;
            } else if (name == "ekf.measurement_noise_v") {
                rv = param.as_double();
                measurement_noise_updated = true;
            } else if (name == "ekf.measurement_noise_w") {
                rw = param.as_double();
                measurement_noise_updated = true;
            } else if (name == "viz.velocity_scale") {
                marker_velocity_scale_ = std::max(param.as_double(), 0.0);
            } else if (name == "ekf.max_acceleration") {
                max_acceleration_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "max_acceleration updated to %.2f", max_acceleration_);
            } else if (name == "ekf.accel_lowpass_alpha") {
                accel_lowpass_alpha_ = std::clamp(param.as_double(), 0.01, 1.0);
                RCLCPP_INFO(this->get_logger(), "accel_lowpass_alpha updated to %.3f", accel_lowpass_alpha_);
            }
        }

        if (measurement_noise_updated) {
            ekf_.setMeasurementNoise(ru, rv, rw);
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "parameters accepted";
        return result;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        initTuningGui();
        applyTuningFromGui();

        cv::Mat image = imageMsgToMat(msg);
        if (image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image.");
            return;
        }

        std::vector<Detection> detections;
        detector_.detect(image, detections);

        MeasurementResult measurement;
        updateEkfByDetection(detections, measurement);

        const cv::Point3f pred = ekf_.getPosition();
        const cv::Point3f velocity = ekf_.getVelocity();
        const cv::Point3f acceleration = ekf_.getAcceleration();
        showDetectionImage(image, detections, pred, velocity, acceleration);

        const rclcpp::Time stamp = this->get_clock()->now();
        visualization_msgs::msg::MarkerArray markers;
        appendObservationMarkers(detections, stamp, markers);
        appendPredictionMarkers(pred, velocity, stamp, markers);
        marker_pub_->publish(markers);

        showTuningView(image, pred, velocity);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "camera_frame";
        pose_msg.header.stamp = stamp;
        pose_msg.pose.position = toGeometryPoint(pred.x, pred.y, pred.z);
        pose_msg.pose.orientation.w = 1.0;
        ekf_pub_->publish(pose_msg);

        publishTargetState(pose_msg.header, pred, velocity, acceleration, measurement);
    }

public:
    explicit ArmorDetectEkfNode(const std::string & name)
    : Node(name), detector_(
        (std::filesystem::path(ament_index_cpp::get_package_share_directory("armor_detect_ekf")) /
            "models" / "best.onnx").string(),
        true)
    {
        RCLCPP_INFO(this->get_logger(), "%s has been started.", name.c_str());

        ekf_.setCameraParams(fx_, cx_, cy_, armor_w_);

        const double p0 = this->declare_parameter<double>("ekf.initial_covariance_scale", 100.0);
        const double q = this->declare_parameter<double>("ekf.process_noise_scale", 0.05);
        const double ru = this->declare_parameter<double>("ekf.measurement_noise_u", 16.0);
        const double rv = this->declare_parameter<double>("ekf.measurement_noise_v", 16.0);
        const double rw = this->declare_parameter<double>("ekf.measurement_noise_w", 64.0);

        max_acceleration_ = this->declare_parameter<double>("ekf.max_acceleration", 50.0);
        accel_lowpass_alpha_ = this->declare_parameter<double>("ekf.accel_lowpass_alpha", 0.3);

        marker_velocity_scale_ = this->declare_parameter<double>("viz.velocity_scale", 0.20);
        enable_tuning_gui_ = this->declare_parameter<bool>("viz.enable_tuning_gui", true);
        gui_window_name_ = this->declare_parameter<std::string>("viz.tuning_window_name", "ekf_tuning");
        show_detection_image_ = this->declare_parameter<bool>("viz.show_detection_image", false);
        detection_window_name_ = this->declare_parameter<std::string>("viz.detection_window_name", "armor_detection");
        draw_prediction_on_image_ = this->declare_parameter<bool>("viz.draw_prediction_on_image", true);
        draw_ballistic_on_image_ = this->declare_parameter<bool>("viz.draw_ballistic_on_image", true);
        ballistic_bullet_speed_ = this->declare_parameter<double>("viz.ballistic_bullet_speed", 20.0);
        ballistic_gravity_ = this->declare_parameter<double>("viz.ballistic_gravity", 9.81);
        ballistic_delay_ = this->declare_parameter<double>("viz.ballistic_delay", 0.1);

        ekf_.setInitialCovarianceScale(p0);
        ekf_.setProcessNoiseScale(q);
        ekf_.setMeasurementNoise(ru, rv, rw);

        tb_p0_ = static_cast<int>(std::round(std::clamp(p0, 0.0, 1000.0)));
        tb_q_ = static_cast<int>(std::round(std::clamp(q * 1000.0, 0.0, 1000.0)));
        tb_ru_ = static_cast<int>(std::round(std::clamp(ru, 0.0, 500.0)));
        tb_rv_ = static_cast<int>(std::round(std::clamp(rv, 0.0, 500.0)));
        tb_rw_ = static_cast<int>(std::round(std::clamp(rw, 0.0, 1000.0)));
        tb_viz_vel_ = static_cast<int>(std::round(std::clamp(marker_velocity_scale_ * 100.0, 0.0, 200.0)));
        tb_max_accel_ = static_cast<int>(std::round(std::clamp(max_acceleration_, 1.0, 200.0)));
        tb_accel_alpha_ = static_cast<int>(std::round(std::clamp(accel_lowpass_alpha_ * 100.0, 1.0, 100.0)));

        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ArmorDetectEkfNode::onParameterUpdate, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "OriginalImage",
            rclcpp::SensorDataQoS(),
            std::bind(&ArmorDetectEkfNode::imageCallback, this, _1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_markers", 10);
        ekf_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf_position", 10);
        target_state_pub_ = this->create_publisher<armor_detect_ekf::msg::TargetState>("target_state", 10);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorDetectEkfNode>("detect_ekf_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
