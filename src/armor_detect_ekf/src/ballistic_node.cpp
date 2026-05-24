#include "rclcpp/rclcpp.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "armor_detect_ekf/msg/target_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class BallisticNode : public rclcpp::Node
{
private:
    struct Vec3
    {
        double x;
        double y;
        double z;
    };

    struct AimSolution
    {
        double yaw;
        double pitch;
        double time;
    };

    const std::vector<double> muzzle_position_param_ =
        this->declare_parameter<std::vector<double>>("muzzle_position_world", {0.0, 0.0, 0.0});
    const std::vector<double> muzzle_rpy_param_ =
        this->declare_parameter<std::vector<double>>("muzzle_rpy_world", {0.0, 0.0, 0.0});
    const double bullet_initial_speed_ = this->declare_parameter<double>("bullet_initial_speed", 20.0);
    const double gravity_ = this->declare_parameter<double>("gravity", 9.81);
    const double delay_ = this->declare_parameter<double>("delay", 0.1);

    rclcpp::Subscription<armor_detect_ekf::msg::TargetState>::SharedPtr target_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aim_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr aim_solution_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aim_marker_pub_;

    Vec3 muzzle_position_world_{0.0, 0.0, 0.0};
    std::array<std::array<double, 3>, 3> rotation_muzzle_to_world_{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}
    }};

    static Vec3 pointMsgToVec3(const geometry_msgs::msg::Point & msg)
    {
        return Vec3{msg.x, msg.y, msg.z};
    }

    static Vec3 vectorMsgToVec3(const geometry_msgs::msg::Vector3 & msg)
    {
        return Vec3{msg.x, msg.y, msg.z};
    }

    static geometry_msgs::msg::Point vec3ToPointMsg(const Vec3 & v)
    {
        geometry_msgs::msg::Point msg;
        msg.x = v.x;
        msg.y = v.y;
        msg.z = v.z;
        return msg;
    }

    static std::array<std::array<double, 3>, 3> createRotationMatrix(double roll, double pitch, double yaw)
    {
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        return {{
            {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}},
            {{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}},
            {{-sp, cp * sr, cp * cr}}
        }};
    }

    Vec3 rotateWorldVectorToMuzzle(const Vec3 & vec_world) const
    {
        return Vec3{
            rotation_muzzle_to_world_[0][0] * vec_world.x +
            rotation_muzzle_to_world_[1][0] * vec_world.y +
            rotation_muzzle_to_world_[2][0] * vec_world.z,
            rotation_muzzle_to_world_[0][1] * vec_world.x +
            rotation_muzzle_to_world_[1][1] * vec_world.y +
            rotation_muzzle_to_world_[2][1] * vec_world.z,
            rotation_muzzle_to_world_[0][2] * vec_world.x +
            rotation_muzzle_to_world_[1][2] * vec_world.y +
            rotation_muzzle_to_world_[2][2] * vec_world.z
        };
    }

    Vec3 rotateMuzzleVectorToWorld(const Vec3 & vec_muzzle) const
    {
        return Vec3{
            rotation_muzzle_to_world_[0][0] * vec_muzzle.x +
            rotation_muzzle_to_world_[0][1] * vec_muzzle.y +
            rotation_muzzle_to_world_[0][2] * vec_muzzle.z,
            rotation_muzzle_to_world_[1][0] * vec_muzzle.x +
            rotation_muzzle_to_world_[1][1] * vec_muzzle.y +
            rotation_muzzle_to_world_[1][2] * vec_muzzle.z,
            rotation_muzzle_to_world_[2][0] * vec_muzzle.x +
            rotation_muzzle_to_world_[2][1] * vec_muzzle.y +
            rotation_muzzle_to_world_[2][2] * vec_muzzle.z
        };
    }

    Vec3 transformWorldPointToMuzzle(const Vec3 & point_world) const
    {
        const Vec3 relative{
            point_world.x - muzzle_position_world_.x,
            point_world.y - muzzle_position_world_.y,
            point_world.z - muzzle_position_world_.z
        };
        return rotateWorldVectorToMuzzle(relative);
    }

    Vec3 predictTargetAtTime(
        const Vec3 & position,
        const Vec3 & velocity,
        const Vec3 & acceleration,
        double t) const
    {
        return Vec3{
            position.x + velocity.x * t + 0.5 * acceleration.x * t * t,
            position.y + velocity.y * t + 0.5 * acceleration.y * t * t,
            position.z + velocity.z * t + 0.5 * acceleration.z * t * t
        };
    }

    double ballisticResidual(
        double bullet_time,
        const Vec3 & position,
        const Vec3 & velocity,
        const Vec3 & acceleration) const
    {
        // 在 muzzle 坐标系中，Z 朝前(深度)，X 朝右，Y 朝下
        // 子弹受重力下坠 -> 沿 muzzle Y 轴正方向
        const Vec3 target = predictTargetAtTime(position, velocity, acceleration, bullet_time + delay_);
        const double y_with_gravity = target.y + 0.5 * gravity_ * bullet_time * bullet_time;

        return target.x * target.x +
               y_with_gravity * y_with_gravity +
               target.z * target.z -
               bullet_initial_speed_ * bullet_initial_speed_ * bullet_time * bullet_time;
    }

    bool solveAim(
        const Vec3 & position,
        const Vec3 & velocity,
        const Vec3 & acceleration,
        AimSolution & solution) const
    {
        constexpr double kTimeMin = 1e-3;
        constexpr double kTimeMax = 3.0;
        constexpr double kScanStep = 2e-3;
        constexpr int kBisectionIters = 60;
        constexpr double kHorizontalEps = 1e-6;

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
            return false;
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

        const Vec3 target = predictTargetAtTime(position, velocity, acceleration, t_hit + delay_);
        const double horizontal = std::hypot(target.x, target.z);
        if (horizontal < kHorizontalEps) {
            return false;
        }

        // Y = Y_target + 重力的额外下坠; 用 y_with_gravity
        const double y_with_gravity = target.y + 0.5 * gravity_ * t_hit * t_hit;
        solution.yaw = std::atan2(target.x, target.z);
        solution.pitch = std::atan2(y_with_gravity, horizontal);
        solution.time = t_hit;

        return std::isfinite(solution.yaw) &&
               std::isfinite(solution.pitch) &&
               std::isfinite(solution.time);
    }

    void publishAimResult(
        const std_msgs::msg::Header & header,
        const Vec3 & hit_world,
        const AimSolution & aim)
    {
        geometry_msgs::msg::PointStamped aim_point_msg;
        aim_point_msg.header = header;
        aim_point_msg.point = vec3ToPointMsg(hit_world);
        aim_point_pub_->publish(aim_point_msg);

        geometry_msgs::msg::Vector3Stamped aim_solution_msg;
        aim_solution_msg.header = header;
        aim_solution_msg.vector.x = aim.yaw;
        aim_solution_msg.vector.y = aim.pitch;
        aim_solution_msg.vector.z = aim.time;
        aim_solution_pub_->publish(aim_solution_msg);

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "aim_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = aim_point_msg.point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = 1.0;
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 0.95;
        marker.lifetime = rclcpp::Duration::from_seconds(0.15);
        aim_marker_pub_->publish(marker);
    }

    void loadFixedMuzzlePose()
    {
        if (muzzle_position_param_.size() != 3 || muzzle_rpy_param_.size() != 3) {
            throw std::runtime_error(
                "Parameters muzzle_position_world and muzzle_rpy_world must each contain 3 values.");
        }

        muzzle_position_world_ = Vec3{
            muzzle_position_param_[0],
            muzzle_position_param_[1],
            muzzle_position_param_[2]
        };
        rotation_muzzle_to_world_ = createRotationMatrix(
            muzzle_rpy_param_[0],
            muzzle_rpy_param_[1],
            muzzle_rpy_param_[2]);

        RCLCPP_INFO(
            this->get_logger(),
            "Use fixed muzzle pose in world | pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
            muzzle_position_world_.x,
            muzzle_position_world_.y,
            muzzle_position_world_.z,
            muzzle_rpy_param_[0],
            muzzle_rpy_param_[1],
            muzzle_rpy_param_[2]);
    }

    void targetStateCallback(const armor_detect_ekf::msg::TargetState::SharedPtr msg)
    {
        if (!msg->measurement_valid) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "No valid target measurement in current frame.");
            return;
        }

        const Vec3 position_world = pointMsgToVec3(msg->position);
        const Vec3 velocity_world = vectorMsgToVec3(msg->velocity);
        const Vec3 acceleration_world = vectorMsgToVec3(msg->acceleration);

        const Vec3 position_muzzle = transformWorldPointToMuzzle(position_world);
        const Vec3 velocity_muzzle = rotateWorldVectorToMuzzle(velocity_world);
        const Vec3 acceleration_muzzle = rotateWorldVectorToMuzzle(acceleration_world);

        AimSolution aim{};
        if (!solveAim(position_muzzle, velocity_muzzle, acceleration_muzzle, aim)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 500, "No valid ballistic solution in time window.");
            return;
        }

        const Vec3 hit_muzzle = predictTargetAtTime(
            position_muzzle, velocity_muzzle, acceleration_muzzle, aim.time + delay_);
        const Vec3 hit_world_vec = rotateMuzzleVectorToWorld(hit_muzzle);
        const Vec3 hit_world{
            muzzle_position_world_.x + hit_world_vec.x,
            muzzle_position_world_.y + hit_world_vec.y,
            muzzle_position_world_.z + hit_world_vec.z
        };

        publishAimResult(msg->header, hit_world, aim);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            100,
            "Aim solution | yaw=%.4f rad pitch=%.4f rad t=%.4f s hit=(%.3f, %.3f, %.3f)",
            aim.yaw,
            aim.pitch,
            aim.time,
            hit_world.x,
            hit_world.y,
            hit_world.z);
    }

public:
    BallisticNode() : Node("ballistic_node")
    {
        loadFixedMuzzlePose();

        target_state_sub_ = this->create_subscription<armor_detect_ekf::msg::TargetState>(
            "target_state", 10, std::bind(&BallisticNode::targetStateCallback, this, _1));

        aim_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("aim_point", 10);
        aim_solution_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("aim_solution", 10);
        aim_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("aim_point_marker", 10);

        RCLCPP_INFO(this->get_logger(), "ballistic_node has been started.");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BallisticNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
