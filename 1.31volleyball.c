/*
 * File: rm2024_engineering.c
 * Description: Robot arm volleyball hit demo
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#define TIME_STEP 64
#define JOINT_NUM 6
#define pi 3.1415926f

// 连杆长度（按你原来）
#define L1 0.2f
#define L2 0.2f
#define L3 0.1f
#define L4 0.06f
int pick_pose_id=0;
int pick_timer = 0;
typedef struct {
  double set;
  double now;
  double out;
} joint_s;

/* ===================== 逆解函数 ===================== */
float clamp(float x)
{
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

void i_kin(float *set_pos6, joint_s *joints, int aba)
{
    float X = set_pos6[0];
    float Y = set_pos6[1];
    float Z = set_pos6[2];
    float aR = set_pos6[3];
    float aP = set_pos6[4];
    float aY = set_pos6[5];

    float d1, t2, t3, t4, t5, t6;

    t5 = aP;
    t6 = aR;

    d1 = Z - L4 * sinf(aP);

    // 直线关节限幅
    if (d1 > 0.2f) d1 = 0.2f;
    if (d1 < 0.0f) d1 = 0.0f;

    float l = L4 * cosf(aP);

    float x4 = X - l * cosf(aY) - L3 * cosf(aY);
    float y4 = Y - l * sinf(aY) - L3 * sinf(aY);

    float r = sqrtf(x4 * x4 + y4 * y4);
    if (r < 1e-4f) r = 1e-4f;

    float c3 = (L1*L1 + L2*L2 - r*r) / (2*L1*L2);
    c3 = clamp(c3);
    t3 = ((aba == 'V') ? 1.0f : -1.0f) * (pi - acosf(c3));

    float t_o23 = atan2f(y4, x4);

    float c2 = (L1*L1 + r*r - L2*L2) / (2*L1*r);
    c2 = clamp(c2);
    float t2_ = acosf(c2);

    t2 = t_o23 + ((aba == 'V') ? -1.0f : 1.0f) * t2_;

    t4 = aY - t2 - t3;

    joints[0].set = d1;
    joints[1].set = t2;
    joints[2].set = t3;
    joints[3].set = t4;
    joints[4].set = t5;
    joints[5].set = t6;
}

int arm_reached(joint_s *j)
{
    for (int i = 0; i < JOINT_NUM; i++)
        if (fabs(j[i].now - j[i].set) > 0.02)
            return 0;
    return 1;
}

/* ===================== 状态机 ===================== */
typedef enum {
    ARM_READY,
    ARM_SWING,
    ARM_RETURN
} arm_state_t;


arm_state_t arm_state = ARM_READY;

#define PICK_POSE_NUM 4 //新增
double pick_action[PICK_POSE_NUM][JOINT_NUM] = {
    { 0.2,  0.0,  0.2,  0.0,  0.0,  0.0 }, 
    { 0.2,  -0.24,  0.2,  0.120,  -2.16,  1.680 },
    { 0.2,  -0.24,  0.2,  0.120,  0.24,  1.680 },  
    { 0.2,  -0.24,  0.2,  0.120,  -2.16,  1.680}, 
};

/* ===================== 主函数 ===================== */
int main(int argc, char **argv)
{
    wb_robot_init();

    char motor_names[JOINT_NUM][16] = {
        "joint1motor","joint2motor","joint3motor",
        "joint4motor","joint5motor","joint6motor"
    };
    char sensor_names[JOINT_NUM][16] = {
        "joint1sensor","joint2sensor","joint3sensor",
        "joint4sensor","joint5sensor","joint6sensor"
    };

    WbDeviceTag motor_tags[JOINT_NUM];
    WbDeviceTag sensor_tags[JOINT_NUM];

    for (int i = 0; i < JOINT_NUM; i++) {
        motor_tags[i] = wb_robot_get_device(motor_names[i]);
        sensor_tags[i] = wb_robot_get_device(sensor_names[i]);
        wb_position_sensor_enable(sensor_tags[i], TIME_STEP);
        wb_motor_set_position(motor_tags[i], 0);
    }

    joint_s joints[JOINT_NUM] = {0};
    float set_pos6[6];

    /* 五点击球轨迹 */
    float traj[5][6] = {
        {0.30f, -0.15f, 0.22f, 0, -pi/2, 0}, // ready
        {0.26f, -0.20f, 0.22f, 0, -pi/2, 0}, // pre_swing
        {0.38f,  0.00f, 0.22f, 0, -pi/2, 0}, // strike
        {0.42f,  0.05f, 0.22f, 0, -pi/2, 0}, // follow
        {0.28f, -0.15f, 0.28f, 0, -pi/2, 0}  // back
    };

    int traj_id = 0;

    while (wb_robot_step(TIME_STEP) != -1)
{
    /* 读传感器 */
    for (int i = 0; i < JOINT_NUM; i++)
        joints[i].now = wb_position_sensor_get_value(sensor_tags[i]);

    /* ===== 取矿动作组模式 ===== */
    for (int i = 0; i < JOINT_NUM; i++) {
        joints[i].set = pick_action[pick_pose_id][i];
        wb_motor_set_position(motor_tags[i], joints[i].set); // ★真正控制电机
    }

    pick_timer++;

    /* 每隔一段时间切换到下一个动作 */
    if (pick_timer > 30) {   // 30个周期 ≈ 2秒（64ms * 30）
        pick_timer = 0;
        pick_pose_id++;

        if (pick_pose_id >= PICK_POSE_NUM)
            pick_pose_id = PICK_POSE_NUM - 1;  // 停在最后一个动作
    }
}


    wb_robot_cleanup();
    return 0;
}
