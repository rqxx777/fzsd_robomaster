#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // 合理的速度值
        linear_vel_ = 0.2;    // 降低线速度：0.2 m/s
        angular_vel_ = 0.5;   // 降低角速度：0.5 rad/s
        
        // 保存终端设置
        tcgetattr(STDIN_FILENO, &old_settings_);
        termios new_settings = old_settings_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
        
        std::cout << "\n=== 移动机器人键盘控制 ===\n";
        std::cout << "W: 前进 (" << linear_vel_ << " m/s)\n";
        std::cout << "S: 后退 (" << linear_vel_ << " m/s)\n";
        std::cout << "A: 左转 (" << angular_vel_ << " rad/s)\n";
        std::cout << "D: 右转 (" << angular_vel_ << " rad/s)\n";
        std::cout << "X: 停止\n";
        std::cout << "Q: 退出程序\n";
        std::cout << "==========================\n\n";
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardTeleop::timer_callback, this));
    }
    
    ~KeyboardTeleop()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings_);
    }
    
private:
    void timer_callback()
    {
        char key = get_key();
        
        auto twist = geometry_msgs::msg::Twist();
        
        switch(key)
        {
            case 'w': case 'W':
                twist.linear.x = linear_vel_;
                std::cout << "前进: " << linear_vel_ << " m/s\n";
                break;
            case 's': case 'S':
                twist.linear.x = -linear_vel_;
                std::cout << "后退: " << linear_vel_ << " m/s\n";
                break;
            case 'a': case 'A':
                twist.angular.z = angular_vel_;
                std::cout << "左转: " << angular_vel_ << " rad/s\n";
                break;
            case 'd': case 'D':
                twist.angular.z = -angular_vel_;
                std::cout << "右转: " << angular_vel_ << " rad/s\n";
                break;
            case 'x': case 'X':
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                std::cout << "停止\n";
                break;
            case 'q': case 'Q':
                std::cout << "退出程序\n";
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_pub_->publish(twist);
                rclcpp::shutdown();
                return;
            default:
                return;
        }
        
        cmd_pub_->publish(twist);
    }
    
    char get_key()
    {
        fd_set set;
        struct timeval timeout;
        
        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;  // 非阻塞
        
        if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0)
        {
            return getchar();
        }
        return 0;
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    termios old_settings_;
    double linear_vel_;
    double angular_vel_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    rclcpp::spin(node);
    return 0;
}
