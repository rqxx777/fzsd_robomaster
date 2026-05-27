#ifndef SERIAL_PACKET_H
#define SERIAL_PACKET_H

#include <cstdint>
#include <algorithm>

#pragma pack(1)   // 1字节对齐，保证结构体大小与协议一致

#define visionMsg inf_visionMsg
#define robotMsg inf_robotMsg
//#define VelMsg Vel_visionMsg

// 下行数据帧: 视觉 → MCU (发送瞄准指令和底盘速度)
struct inf_visionMsg
{
    uint16_t head;         // 帧头 0xA5
    uint8_t fire;          // 开火标志 (0/1)
    uint8_t tracking;      // 跟踪标志 (0/1)
    uint8_t vel_control;   // 速度控制标志
    float aimYaw;          // 目标Yaw角度 (度)
    float aimPitch;        // 目标Pitch角度 (度)
    float aimPVel;         // 目标Pitch角速度 (度/秒)
    float aimYVel;         // 目标Yaw角速度 (度/秒)
    float vx;              // 底盘X方向线速度
    float vy;              // 底盘Y方向线速度
    float wz;              // 底盘Z方向角速度
    uint8_t gyroscope;     // 陀螺仪控制标志
};

// 上行数据帧: MCU → 视觉 (发送机器人状态和比赛信息)
struct inf_robotMsg
{
    uint16_t head;                  // 帧头 0xA5
    uint8_t mode;                   // 机器人模式
    uint8_t foeColor;               // 敌方颜色: 0=blue, 1=red
    float robotYaw;                 // 云台当前Yaw角 (度)
    float robotPitch;               // 云台当前Pitch角 (度)
    float muzzleSpeed;              // 弹丸初速 (m/s)
    float bigYaw;                   // 大Yaw (云台总Yaw)
    int current_robot_quantity;     // 当前存活机器人数量
    int blood_warn_state;           // 血量警告状态
    int game_process;               // 比赛进程状态 (4=比赛中)
    float current_state_left_time;  // 当前阶段剩余时间 (秒)
    uint8_t is_success_ourpreempt;   // 我方成功占点标志 (0/1)
    uint8_t is_success_enemypreempt; // 敌方成功占点标志 (0/1)
};

// struct Vel_visionMsg
// {
//     uint16_t head;
//     uint8_t fire;     // 开火标志
//     uint8_t tracking; // 跟踪标志
//     uint8_t VelControl; // 速度控制标志
//     float aimYaw;     // 目标Yaw
//     float aimPitch;   // 目标Pitch
//     float aimPVel;      // 目标Pitch速度
//     float aimYVel;      // 目标Yaw速度
// };

// 共用体: 方便以字节数组形式操作 visionMsg
union visionArray
{
    struct visionMsg msg;
    uint8_t array[sizeof(struct visionMsg)];
};

// union VelArray
// {
//     struct VelMsg msg;
//     uint8_t array[sizeof(struct VelMsg)];
// };

// 共用体: 方便以字节数组形式操作 robotMsg
union robotArray
{
    struct robotMsg msg;
    uint8_t array[sizeof(struct robotMsg)];
};

#pragma pack()

#endif // SERIAL_PACKET_H
