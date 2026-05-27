
#include "vision_attacker/vision_attacker_node.hpp"
#include "vision_attacker/planner.hpp"
#include "vision_attacker/vision_attacker_node.hpp"

#include <algorithm>

// Tracker_node 构造函数实现
  // 构造函数: 加载弹道/火控/规划器参数，初始化发布者和订阅者
Tracker_node::Tracker_node(std::string node_name) : rclcpp::Node(node_name)
{
    lagTime = declare_parameter("trajectory.lag_time", 0.08);
    airK = declare_parameter("trajectory.air_k", 0.04);
    yawFix = declare_parameter("trajectory.yaw_fix", 1.0);
    pitchFix = declare_parameter("trajectory.pitch_fix", 0.0);
    pitchFixPerMeter = declare_parameter("trajectory.pitch_fix_per_meter", -0.25);
    pitchFixMin = declare_parameter("trajectory.pitch_fix_min", -1.3);
    carThreshold = declare_parameter("fire_ctrl.car_attack_threshold", 30.0);
    yawThreshold = declare_parameter("fire_ctrl.yaw_attack_threshold", carThreshold);
    pitchThreshold = declare_parameter("fire_ctrl.pitch_attack_threshold", carThreshold);
    outpostThreshold = declare_parameter("fire_ctrl.outpost_attack_threshold", 5.0);
    thresholdFix = declare_parameter("fire_ctrl.threshold_fix", 0.0);

    //solver
    double max_yaw_acc = declare_parameter("solver.max_yaw_acc", 100.0);
    double max_pitch_acc = declare_parameter("solver.max_pitch_acc", 100.0);
    std::vector<double> Q_yaw_v = declare_parameter("solver.Q_yaw", std::vector<double>{0.1});
    std::vector<double> R_yaw_v = declare_parameter("solver.R_yaw", std::vector<double>{0.1});
    std::vector<double> Q_pitch_v = declare_parameter("solver.Q_pitch", std::vector<double>{0.1});
    std::vector<double> R_pitch_v = declare_parameter("solver.R_pitch", std::vector<double>{0.1});
    //planner
    double yaw_offset_ = declare_parameter("planner.yaw_offset", 0.0);
    double pitch_offset_ = declare_parameter("planner.pitch_offset", 0.0);
    double fire_thresh_ = declare_parameter("planner.fire_thresh", 0.0);
    double low_speed_delay_time_ = declare_parameter("planner.low_speed_delay_time", 0.0);
    double high_speed_delay_time_ = declare_parameter("planner.high_speed_delay_time", 0.0);
    double decision_speed_ = declare_parameter("planner.decision_speed", 0.0);

    planner_ = std::make_unique<auto_aim::Planner>(
        yaw_offset_, pitch_offset_, fire_thresh_, low_speed_delay_time_, high_speed_delay_time_, decision_speed_,
        max_yaw_acc, max_pitch_acc, Q_yaw_v, R_yaw_v, Q_pitch_v, R_pitch_v,
        lagTime, airK);
    
    robotPtr = std::make_unique<vision_interfaces::msg::Robot>();
    markerPub = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    aimMarkerPub = this->create_publisher<visualization_msgs::msg::Marker>("/real_aiming_point", 10);
    aimPub = create_publisher<vision_interfaces::msg::AutoAim>(
        "/serial_driver/aim_target", rclcpp::SensorDataQoS());
    robotSub = create_subscription<vision_interfaces::msg::Robot>(
        "/serial_driver/robot", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::robot_callback, this, std::placeholders::_1));
    targetSub = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::target_callback, this, std::placeholders::_1));

    aimPoint.header.frame_id = "aim_odom";
    aimPoint.ns = "aiming_point";
    aimPoint.type = visualization_msgs::msg::Marker::SPHERE;
    aimPoint.action = visualization_msgs::msg::Marker::ADD;
    aimPoint.scale.x = aimPoint.scale.y = aimPoint.scale.z = 0.12;
    aimPoint.color.r = 1.0;
    aimPoint.color.g = 1.0;
    aimPoint.color.b = 1.0;
    aimPoint.color.a = 1.0;
    aimPoint.lifetime = rclcpp::Duration::from_seconds(0.1);

    realAimPoint.header.frame_id = "aim_odom";
    realAimPoint.ns = "real_aiming_point";
    realAimPoint.type = visualization_msgs::msg::Marker::SPHERE;
    realAimPoint.action = visualization_msgs::msg::Marker::ADD;
    realAimPoint.scale.x = realAimPoint.scale.y = realAimPoint.scale.z = 0.12;
    realAimPoint.color.r = 1.0;
    realAimPoint.color.g = 0.0;
    realAimPoint.color.b = 0.0;
    realAimPoint.color.a = 1.0;
    realAimPoint.lifetime = rclcpp::Duration::from_seconds(0.1);
    RCLCPP_INFO(get_logger(), "Vision Attacker Node Initialized.");
}

void Tracker_node::robot_callback(const vision_interfaces::msg::Robot robot)
{
  // 缓存机器人自身状态 (云台角度 + 弹速)
    try
    {
        *robotPtr = robot;
    }
    catch (std::exception &ex)
    {
        RCLCPP_ERROR(get_logger(), "获取机器人信息时发生错误.");
    }
}

void Tracker_node::target_callback(const auto_aim_interfaces::msg::Target target_msg)
  // 核心决策: 动态调参 → 弹道解算 → MPC规划 → 火控判断 → 发布瞄准指令
{
    lagTime = get_parameter("trajectory.lag_time").as_double();
    airK = get_parameter("trajectory.air_k").as_double();
    yawFix = get_parameter("trajectory.yaw_fix").as_double();
    pitchFix = get_parameter("trajectory.pitch_fix").as_double();
    pitchFixPerMeter = get_parameter("trajectory.pitch_fix_per_meter").as_double();
    pitchFixMin = get_parameter("trajectory.pitch_fix_min").as_double();
    carThreshold = get_parameter("fire_ctrl.car_attack_threshold").as_double();
    yawThreshold = get_parameter("fire_ctrl.yaw_attack_threshold").as_double();
    pitchThreshold = get_parameter("fire_ctrl.pitch_attack_threshold").as_double();
    outpostThreshold = get_parameter("fire_ctrl.outpost_attack_threshold").as_double();
    thresholdFix = get_parameter("fire_ctrl.threshold_fix").as_double();
    // //traj old
    // Eigen::Vector2d xy;
    // xy << target_msg.position.x, target_msg.position.y;
    // double yDis = target_msg.position.z;
    // double xDis = xy.norm();
    // double flyTime1 = 0;

    // Prepare default aim message
    vision_interfaces::msg::AutoAim aim;
    aim.aim_yaw = robotPtr->self_yaw;
    aim.aim_pitch = robotPtr->self_pitch;
    aim.fire = 0;

    if (!target_msg.tracking)
    {
        aim.tracking = 0;
        aimPub->publish(aim);

        return;
    }

    // Convert tracker message to planner Target and run planner
        auto_aim::Target t;
        t.from_msg(target_msg, std::chrono::steady_clock::now());
        double bullet_speed = robotPtr->muzzle_speed ;
        auto plan = planner_->plan(t, bullet_speed);

        // Map planner output (radians) to message (degrees)
        aim.aim_yaw = plan.target_yaw * 180.0 / M_PI;
         aim.aim_pitch = plan.target_pitch * 180.0 / M_PI;
        // // old traj pitch
        // solveTrajectory(flyTime1, xDis, yDis, lagTime, bullet_speed, airK);
        // aim.aim_pitch = solveTrajectory(flyTime1, xDis, yDis, lagTime, bullet_speed, airK) * 180.0 / M_PI;
        aim.fire = 0; // 将在后面根据aim与self的误差重新判断
        aim.tracking = 1;
        

        // Update markers from planner debug info
        aimPoint.pose.position.x = planner_->debug_xyza[0];
        aimPoint.pose.position.y = planner_->debug_xyza[1];
        aimPoint.pose.position.z = planner_->debug_xyza[2];

        realAimPoint.pose.position.x = target_msg.position.x - target_msg.radius_1 * cos(robotPtr->self_yaw / 180.0 * M_PI);
        realAimPoint.pose.position.y = target_msg.position.y - target_msg.radius_1 * sin(robotPtr->self_yaw / 180.0 * M_PI);
        realAimPoint.pose.position.z = target_msg.position.z;

        aimPoint.header.stamp = now();
        markerPub->publish(aimPoint);
        aimMarkerPub->publish(realAimPoint);
        // 动态调整yaw和pitch修正，距离越远修正越大，近距离则不修正        
        // const double distance_xy = std::hypot(target_msg.position.x, target_msg.position.y);
        // if (distance_xy > 3.4) {
        //     pitchFix -= ((distance_xy - 3.7)+1); // 根据距离动态调整yaw修正
        // }

        //动态火控
        const double distance_xy = std::hypot(target_msg.position.x, target_msg.position.y);
        if (distance_xy > 3.5)
        {
            yawThreshold = 1.0;
        }
        else
        {
            yawThreshold = 5.0 -distance_xy;
            if (yawThreshold < 1.0)
            {
                yawThreshold = 1.0;
            }
        }
        pitchThreshold = pitchThreshold + (2.5-distance_xy);
        if ( distance_xy>3.5)
        {
            pitchThreshold =  get_parameter("fire_ctrl.pitch_attack_threshold").as_double();
        }
        aim.aim_yaw += yawFix;
        // const double pitch_fix_dynamic = std::clamp(
        //     pitchFix + pitchFixPerMeter * distance_xy,
        //     pitchFixMin,
        //     std::max(pitchFix, pitchFixMin));
        // aim.aim_pitch += pitch_fix_dynamic;
        // 归一化 aim_yaw 到 [-180, 180)
        while (aim.aim_yaw >= 180.0) aim.aim_yaw -= 360.0;
        while (aim.aim_yaw < -180.0) aim.aim_yaw += 360.0;
        aim.aim_p_vel = plan.pitch_vel * 180.0 / M_PI;
        aim.aim_y_vel = plan.yaw_vel * 180.0 / M_PI;

        // 用解算后的aim角度与云台实际角度的误差判断开火
        double yaw_diff = aim.aim_yaw - robotPtr->self_yaw;
        // 处理yaw角度环绕 (0~360)
        if (yaw_diff > 180.0) yaw_diff -= 360.0;
        if (yaw_diff < -180.0) yaw_diff += 360.0;
        double pitch_diff = aim.aim_pitch - robotPtr->self_pitch;
    // yaw和pitch分别都小于各自阈值才开火
    aim.fire = (std::abs(yaw_diff) < yawThreshold && std::abs(pitch_diff) < pitchThreshold) ? 1 : 0;

        aimPub->publish(aim);
    
    RCLCPP_INFO(get_logger(), "aimYaw:%05.2f/aimPitch:%05.2f/selfYaw:%05.2f/selfPitch:%05.2f/fire:%d/tracking:%d", aim.aim_yaw, aim.aim_pitch, robotPtr->self_yaw, robotPtr->self_pitch, aim.fire, aim.tracking);
    
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tracker_node>("vision_attacker");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
