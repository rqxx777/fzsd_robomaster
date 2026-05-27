#ifndef VISION_SERIAL_DRIVER_HPP
#define VISION_SERIAL_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <vision_interfaces/msg/game_state.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "avgFilter.hpp"
#include "packet.h"

using namespace drivers::serial_driver;
using namespace std::chrono_literals;

// 串口驱动 ROS2 节点: 负责与下位机 (MCU) 的双向串口通信
class serial_driver_node : public rclcpp::Node
{
public:
    /*
    @brief 串口驱动节点构造函数
    @param[in] device_name 串口设备路径 (如 /dev/ttyACM0)
    @param[in] node_name 节点名称
    */
    serial_driver_node(std::string device_name, std::string node_name);

    /*@brief 串口驱动节点析构函数，关闭串口*/
    ~serial_driver_node();

private:
    /*@brief 串口重连定时器回调: 每1秒检测一次串口状态，断开则尝试重连*/
    void serial_reopen_callback();

    /*@brief 独立的串口读取线程: 循环读取下位机发送的机器人状态*/
    void serial_read_thread();

    /*@brief 串口数据写入 (原始字节)*/
    void serial_write(uint8_t *data, size_t len);

    /*@brief 串口写入定时器回调: 每2ms发送一次瞄准数据到下位机*/
    void serial_write_callback();

    /*@brief 导航速度回调: 接收并转发底盘速度指令到下位机*/
    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr Msg);

    /*@brief 自瞄指令回调: 接收瞄准角度并立即通过串口发送给MCU*/
    void auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg);

    /*@brief 机器人状态发布定时器回调: 每2ms发布一次机器人状态和TF变换*/
    void robot_callback();

    visionArray *vArray;          // 下行数据 (发送至MCU)
    robotArray *rArray;           // 上行数据 (接收自MCU)
    avgFilter muzzleSpeedFilter;  // 弹速滑动平均滤波器
    bool isOpen = false;          // 串口连接状态
    std::string *dev_name;        // 串口设备名
    std::thread serialReadThread; // 串口读取线程
    SerialPortConfig *portConfig; // 串口配置 (波特率/流控/校验等)
    IoContext ctx;                // 串口IO上下文
    SerialDriver serialDriver = SerialDriver(ctx);
    uint8_t VelControl;           // 速度控制标志
    int color = 0;                // 我方颜色

    // // Param client to set detect_color
    // using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    // bool initial_set_param_ = false;
    // uint8_t previous_receive_color_ = 0;
    // rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    // ResultFuturePtr et_param_future_;


    // 广播 odom → gimbal_link 的 TF 坐标变换
    double timestamp_offset_ = 0;  // 时间戳偏移补偿
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr reopenTimer;    // 重连定时器 (1Hz)
    rclcpp::TimerBase::SharedPtr writeTimer;     // 写入定时器 (500Hz)
    rclcpp::TimerBase::SharedPtr publishTimer;   // 发布定时器 (500Hz)

    rclcpp::Publisher<vision_interfaces::msg::Robot>::SharedPtr publisher;          // 机器人状态发布
    rclcpp::Publisher<vision_interfaces::msg::Robot>::SharedPtr aimpublisher;       // 瞄准调试发布
    rclcpp::Publisher<vision_interfaces::msg::GameState>::SharedPtr gameStatePublisher; // 比赛状态发布
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr navpublisher;                // 导航指令发布
    rclcpp::Subscription<vision_interfaces::msg::AutoAim>::SharedPtr autoAimSub;    // 瞄准指令订阅
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navSub;              // 导航速度订阅
};

#endif // VISION_SERIAL_DRIVER_HPP