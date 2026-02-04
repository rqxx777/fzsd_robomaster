#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>

class WheelSimulator : public rclcpp::Node
{
public:
    WheelSimulator() : Node("wheel_simulator")
    {
        // 机器人参数
        wheel_radius_ = 0.07;          // 车轮半径 (米)
        wheel_separation_ = 0.3;       // 两轮间距 (米)
        
        // 初始化状态
        left_wheel_angle_ = 0.0;
        right_wheel_angle_ = 0.0;
        last_cmd_time_ = this->now();
        
        // 订阅速度命令
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&WheelSimulator::cmd_callback, this, std::placeholders::_1));
        
        // 发布关节状态
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);
        
        // 定时发布关节状态 (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 50ms = 20Hz
            std::bind(&WheelSimulator::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "车轮模拟器启动");
        RCLCPP_INFO(this->get_logger(), "车轮半径: %.3f m, 轮间距: %.3f m", 
                   wheel_radius_, wheel_separation_);
    }
    
private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 记录收到命令的时间
        last_cmd_time_ = this->now();
        last_cmd_ = *msg;
        
        // 调试输出
        static int count = 0;
        if (count++ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "收到速度命令: 线速度=%.2f m/s, 角速度=%.2f rad/s", 
                       msg->linear.x, msg->angular.z);
        }
    }
    
    void timer_callback()
    {
        auto now = this->now();
        
        // 计算时间差（秒）
        double dt = (now - last_update_time_).seconds();
        
        // 限制时间间隔，防止过大
        if (dt > 0.1) dt = 0.05;  // 如果间隔太大，使用固定值
        
        // 更新上次更新时间
        last_update_time_ = now;
        
        // 如果太久没有收到命令，停止运动
        double time_since_last_cmd = (now - last_cmd_time_).seconds();
        if (time_since_last_cmd > 0.5) {
            // 超过0.5秒没收到命令，停止
            last_cmd_.linear.x = 0.0;
            last_cmd_.angular.z = 0.0;
        }
        
        // 差速驱动运动学模型
        double linear = last_cmd_.linear.x;   // 线速度 (m/s)
        double angular = last_cmd_.angular.z; // 角速度 (rad/s)
        
        // 计算车轮线速度 (m/s)
        double left_linear_vel = linear - (angular * wheel_separation_ / 2.0);
        double right_linear_vel = linear + (angular * wheel_separation_ / 2.0);
        
        // 转换为角速度 (rad/s)
        double left_angular_vel = left_linear_vel / wheel_radius_;
        double right_angular_vel = right_linear_vel / wheel_radius_;
        
        // 更新车轮角度（积分）
        left_wheel_angle_ += left_angular_vel * dt;
        right_wheel_angle_ += right_angular_vel * dt;
        
        // 限制角度范围（防止过大）
        if (fabs(left_wheel_angle_) > 2*M_PI) {
            left_wheel_angle_ = fmod(left_wheel_angle_, 2*M_PI);
        }
        if (fabs(right_wheel_angle_) > 2*M_PI) {
            right_wheel_angle_ = fmod(right_wheel_angle_, 2*M_PI);
        }
        
        // 发布关节状态
        publish_joint_state(now);
    }
    
    void publish_joint_state(rclcpp::Time timestamp)
    {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = timestamp;
        joint_state.header.frame_id = "base_link";
        
        // 关节名称
        joint_state.name = {"left_wheel_joint", "right_wheel_joint", "caster_joint"};
        
        // 关节位置（角度）
        joint_state.position = {left_wheel_angle_, right_wheel_angle_, 0.0};
        
        // 关节速度（可选）
        // 计算近似角速度
        double left_vel = (left_wheel_angle_ - last_left_angle_) / 0.05;
        double right_vel = (right_wheel_angle_ - last_right_angle_) / 0.05;
        joint_state.velocity = {left_vel, right_vel, 0.0};
        
        // 保存当前角度用于下次计算速度
        last_left_angle_ = left_wheel_angle_;
        last_right_angle_ = right_wheel_angle_;
        
        joint_pub_->publish(joint_state);
        
        // 调试输出
        static int publish_count = 0;
        if (publish_count++ % 20 == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                        "发布关节状态: 左轮=%.2f rad, 右轮=%.2f rad", 
                        left_wheel_angle_, right_wheel_angle_);
        }
    }
    
    // ROS 2 组件
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 机器人参数
    double wheel_radius_;
    double wheel_separation_;
    
    // 状态变量
    double left_wheel_angle_;
    double right_wheel_angle_;
    double last_left_angle_;
    double last_right_angle_;
    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_update_time_ = this->now();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelSimulator>());
    rclcpp::shutdown();
    return 0;
}
