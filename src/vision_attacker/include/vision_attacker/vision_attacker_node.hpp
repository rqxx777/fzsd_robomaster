#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include "vision_attacker/outpost.hpp"
#include <Eigen/Dense>
#include "vision_attacker/target.hpp"
#include "vision_attacker/planner.hpp"
#include <chrono>
#include <vector>


// 弹道解算+火控决策 ROS2 节点: 接收跟踪目标和机器人状态，输出瞄准指令
class Tracker_node : public rclcpp::Node
{
public:
    explicit Tracker_node(std::string node_name);

private:
    // 机器人状态回调: 接收云台当前角度和弹速
    void robot_callback(const vision_interfaces::msg::Robot robot);
    // 跟踪目标回调: 执行弹道解算+MPC运动规划+火控判断
    void target_callback(const auto_aim_interfaces::msg::Target target_msg);

    std::unique_ptr<auto_aim::Planner> planner_;  // MPC运动规划器

    // 弹道参数
    double lagTime;           // 预测延迟时间修正 (秒)
    double airK;              // 空气阻力系数
    double yawFix;            // 输出 Yaw 轴修正量 (度)
    double pitchFix;          // 输出 Pitch 轴修正量 (度)
    double pitchFixPerMeter;  // Pitch 轴每米距离动态补偿量 (度/米)
    double pitchFixMin;       // Pitch 轴最小修正值 (度)

    // 火控参数
    double thresholdFix;          // 角度阈值修正
    double carThreshold;          // 地面机器人装甲板攻击角度阈值 (度)
    double yawThreshold;          // Yaw 轴开火角度阈值 (度)
    double pitchThreshold;        // Pitch 轴开火角度阈值 (度)
    double outpostThreshold;      // 前哨站装甲板攻击角度阈值 (度)

    // 可视化标记
    visualization_msgs::msg::Marker aimPoint;      // 瞄准点 (白色球体)
    visualization_msgs::msg::Marker realAimPoint;  // 实际瞄准点 (红色球体)

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr targetSub;
    std::unique_ptr<vision_interfaces::msg::Robot> robotPtr;  // 机器人状态缓存
    rclcpp::Subscription<vision_interfaces::msg::Robot>::SharedPtr robotSub;
    rclcpp::Publisher<vision_interfaces::msg::AutoAim>::SharedPtr aimPub;   // 瞄准指令发布
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;     // 瞄准点可视化
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aimMarkerPub;  // 实际点可视化
};
