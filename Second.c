/*
 * File:          rm2024_engineering.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define JOINT_NUM 6

#define L1 (0.2f)
#define L2 (0.2f)
#define L3 (0.1f)
#define L4 (0.06f)

#ifndef pi
#define pi 3.14159265358979f
#endif
///////////////////////////////////////////////////////////////////////////////////
#define PICK_POSE_NUM 1 //新增//避免麻烦，就不把pick改成hit了。
double pick_action[PICK_POSE_NUM][JOINT_NUM] = {
    
    { 0.0,  -0.24,  1.92,  1.68,  0.0,  0.0 },  // Pose 0：击球
};

int pick_pose_id = 0;
int pick_timer = 0;
#define PICK_HOLD_TIME 200
int hit_mode = 1;   // 1 = 击球动作模式
//////////////////////////////////////////////////////////////////////////////////////////////
/*库中某些函数返回的错误状态*/
typedef enum
{
    RFL_MATRIX_SUCCESS = 0,         /**< 无错误 */
    RFL_MATRIX_ARGUMENT_ERROR = -1, /**< 一个或多个参数不正确 */
    RFL_MATRIX_LENGTH_ERROR = -2,   /**< 数据缓冲区的长度不正确 */
    RFL_MATRIX_SIZE_MISMATCH = -3,  /**< 矩阵的大小与操作不兼容 */
    RFL_MATRIX_NANINF = -4,         /**< 生成非数字 （NaN） 或无穷大 */
    RFL_MATRIX_SINGULAR = -5,       /**< 如果输入矩阵是奇异且不能反转，则由矩阵反演生成 */
    RFL_MATRIX_TEST_FAILURE = -6    /**< 测试失败  */
} rfl_matrix_status;

/*32 位浮点类型定义*/

/*浮点矩阵结构的实例结构*/
typedef struct
{
    uint16_t numRows; /**< 矩阵的行数 */
    uint16_t numCols; /**< 矩阵的列数 */
    float *pData;     /**< 指向矩阵的数据 */
} rfl_matrix_instance;

void rflMatrixInit(rfl_matrix_instance *S, uint16_t nRows, uint16_t nColumns, float *pData);

rfl_matrix_status rflMatrixAdd(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixSub(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixMult(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                                rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixTrans(const rfl_matrix_instance *pSrc, rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixInverse(const rfl_matrix_instance *src, rfl_matrix_instance *dst);

void printMat(const rfl_matrix_instance *src)
{
    printf("====MAT_START====\r\n");

    for (int i = 0; i < src->numRows; i++)
    {
        for (int j = 0; j < src->numCols; j++)
        {
            printf("%8.4f\t", src->pData[i * src->numCols + j]);
        }
        printf("\r\n");
    }

    printf("=====MAT_END=====\r\n");

    return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct joint
{
    double set;
    double now;
    double out;
} joint_s;

void EularToRotmat(float roll, float pitch, float yaw, rfl_matrix_instance *mat)
{
    if (mat->numCols != 3 || mat->numRows != 3)
        return;

    float cr, cp, cy, sr, sp, sy;

    cr = cosf(roll);
    cp = cosf(pitch);
    cy = cosf(yaw);
    sr = sinf(roll);
    sp = sinf(pitch);
    sy = sinf(yaw);

    mat->pData[0] = cp * cr;
    mat->pData[1] = sy * sp * cr - cy * sr;
    mat->pData[2] = cy * sp * cr + sy * sr;
    mat->pData[3] = cp * sr;
    mat->pData[4] = sy * sp * sr + cy * cr;
    mat->pData[5] = cy * sp * sr - sy * cr;
    mat->pData[6] = -sp;
    mat->pData[7] = sy * cp;
    mat->pData[8] = cy * cp;
}

void RotmatToEular(rfl_matrix_instance *mat, float *eular_angle)
{
    if (mat->numCols != 3 || mat->numRows != 3)
        return;

    float res_r = 0;
    float res_p = 0;
    float res_y = 0;

    float sy = sqrtf(mat->pData[(1 - 1) * 3 + (1 - 1)] * mat->pData[(1 - 1) * 3 + (1 - 1)] +
                     mat->pData[(2 - 1) * 3 + (1 - 1)] * mat->pData[(2 - 1) * 3 + (1 - 1)]);

    if (sy > 0.0001)
    {
        res_r = atan2f(mat->pData[(2 - 1) * 3 + (1 - 1)], mat->pData[(1 - 1) * 3 + (1 - 1)]);
        res_p = atan2f(-mat->pData[(3 - 1) * 3 + (1 - 1)], sy);
        res_y = atan2f(mat->pData[(3 - 1) * 3 + (2 - 1)], mat->pData[(3 - 1) * 3 + (3 - 1)]);
    }

    // if (fabs(mat->pData[(3 - 1) * 3 + (1 - 1)]) >= (1 - 0.001))
    // {
    //     if (mat->pData[(3 - 1) * 3 + (1 - 1)] < 0)
    //     {
    //         res_r = 0;
    //         res_p = pi / 2;
    //         res_y = atan2f(mat->pData[(1 - 1) * 3 + (2 - 1)], mat->pData[(1 - 1) * 3 + (3 - 1)]);
    //     }
    //     else
    //     {
    //         res_r = 0;
    //         res_p = -pi / 2;
    //         res_y = atan2f(-mat->pData[(1 - 1) * 3 + (2 - 1)], -mat->pData[(1 - 1) * 3 + (3 - 1)]);
    //     }
    // }
    // else
    // {
    //     res_p = atan2f(-mat->pData[(3 - 1) * 3 + (1 - 1)],
    //                    sqrtf(mat->pData[(1 - 1) * 3 + (1 - 1)] * mat->pData[(1 - 1) * 3 + (1 - 1)] +
    //                          mat->pData[(2 - 1) * 3 + (1 - 1)] * mat->pData[(2 - 1) * 3 + (1 - 1)]));
    //     float cos_p = cosf(res_p);
    //     res_r = atan2f(mat->pData[(2 - 1) * 3 + (1 - 1)] / cos_p, mat->pData[(1 - 1) * 3 + (1 - 1)] / cos_p);
    //     res_y = atan2f(mat->pData[(3 - 1) * 3 + (2 - 1)] / cos_p, mat->pData[(3 - 1) * 3 + (3 - 1)] / cos_p);
    // }

    eular_angle[0] = res_r;
    eular_angle[1] = res_p;
    eular_angle[2] = res_y;
}

void ModifyTransmat(rfl_matrix_instance *rotmat, float *pos, rfl_matrix_instance *transmat)
{
    if (rotmat->numCols != 3 || rotmat->numRows != 3)
        return;
    if (transmat->numCols != 4 || transmat->numRows != 4)
        return;

    transmat->pData[0] = rotmat->pData[0];
    transmat->pData[1] = rotmat->pData[1];
    transmat->pData[2] = rotmat->pData[2];
    transmat->pData[4] = rotmat->pData[3];
    transmat->pData[5] = rotmat->pData[4];
    transmat->pData[6] = rotmat->pData[5];
    transmat->pData[8] = rotmat->pData[6];
    transmat->pData[9] = rotmat->pData[7];
    transmat->pData[10] = rotmat->pData[8];

    transmat->pData[3] = pos[0];
    transmat->pData[7] = pos[1];
    transmat->pData[11] = pos[2];

    transmat->pData[12] = 0.0f;
    transmat->pData[13] = 0.0f;
    transmat->pData[14] = 0.0f;
    transmat->pData[15] = 1.0f;
}

void TransmatToRotmat(rfl_matrix_instance *transmat, rfl_matrix_instance *rotmat)
{
    if (transmat->numCols != 4 || transmat->numRows != 4)
        return;
    if (rotmat->numCols != 3 || rotmat->numRows != 3)
        return;

    rotmat->pData[0] = transmat->pData[0];
    rotmat->pData[1] = transmat->pData[1];
    rotmat->pData[2] = transmat->pData[2];
    rotmat->pData[3] = transmat->pData[4];
    rotmat->pData[4] = transmat->pData[5];
    rotmat->pData[5] = transmat->pData[6];
    rotmat->pData[6] = transmat->pData[8];
    rotmat->pData[7] = transmat->pData[9];
    rotmat->pData[8] = transmat->pData[10];
}

void TransmatToEular(rfl_matrix_instance *transmat, float *eular_angle)
{
    if (transmat->numCols != 4 || transmat->numRows != 4)
        return;

    rfl_matrix_instance rotmat = {0};
    float rotmat_data[9] = {0};
    rflMatrixInit(&rotmat, 3, 3, rotmat_data);

    TransmatToRotmat(transmat, &rotmat);
    RotmatToEular(&rotmat, eular_angle);
}

void TransmatTo3pos(rfl_matrix_instance *transmat, float *pos)
{
    if (transmat->numCols != 4 || transmat->numRows != 4)
        return;

    pos[0] = transmat->pData[3];
    pos[1] = transmat->pData[7];
    pos[2] = transmat->pData[11];
}

void DhToTransmat(rfl_matrix_instance *transmat, float alpha_im1, float a_im1, float d_i, float theta_i)
{
    if (transmat->numCols != 4 || transmat->numRows != 4)
        return;

    float cos_alpha = cosf(alpha_im1);
    float cos_theta = cosf(theta_i);
    float sin_alpha = sinf(alpha_im1);
    float sin_theta = sinf(theta_i);

    transmat->pData[0] = cos_theta;
    transmat->pData[1] = -sin_theta;
    transmat->pData[2] = 0;
    transmat->pData[3] = a_im1;

    transmat->pData[4] = sin_theta * cos_alpha;
    transmat->pData[5] = cos_theta * cos_alpha;
    transmat->pData[6] = -sin_alpha;
    transmat->pData[7] = -sin_alpha * d_i;

    transmat->pData[8] = sin_theta * sin_alpha;
    transmat->pData[9] = cos_theta * sin_alpha;
    transmat->pData[10] = cos_alpha;
    transmat->pData[11] = cos_alpha * d_i;

    transmat->pData[12] = 0.0f;
    transmat->pData[13] = 0.0f;
    transmat->pData[14] = 0.0f;
    transmat->pData[15] = 1.0f;
}

void i_kin(float *set_pos6, joint_s *joints_value,int aba)
{
    float X = set_pos6[0];
    float Y = set_pos6[1];
    float Z = set_pos6[2];
    float aR = set_pos6[3];
    float aP = set_pos6[4];
    float aY = set_pos6[5];

    float d1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;

    t5 = aP;
    t6 = aR;

    d1 = Z - L4 * sinf(aP);

    float l = L4 * cosf(aP);
    float x4 = X - l * cosf(aY) - L3 * cosf(aY);
    float y4 = Y - l * sinf(aY) - L3 * sinf(aY);

    float r = sqrtf(x4 * x4 + y4 * y4);

    t3 =((aba=='V') ? 1.0f : -1.0f )*(pi - acosf((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2)));
    float t_o23 = atan2(y4, x4);
    float t2_ = acosf((L1 * L1 + r * r - L2 * L2) / (2 * L1 * r));
    t2 = t_o23 + ((aba=='V') ? -1.0f : 1.0f )*t2_;
    t4 = aY - t2 - t3;

    joints_value[0].set = d1;
    joints_value[1].set = t2;
    joints_value[2].set = t3;
    joints_value[3].set = t4;
    joints_value[4].set = t5;
    joints_value[5].set = t6;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
    /* necessary to initialize webots stuff */
    wb_robot_init();

    /*
     * You should declare here WbDeviceTag variables for storing
     * robot devices like this:
     *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
     *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
     */
    char motor_names[JOINT_NUM][16] = {"joint1motor", "joint2motor", "joint3motor",
                                       "joint4motor", "joint5motor", "joint6motor"};
    char sensor_names[JOINT_NUM][16] = {"joint1sensor", "joint2sensor", "joint3sensor",
                                        "joint4sensor", "joint5sensor", "joint6sensor"};
    WbDeviceTag motor_tags[JOINT_NUM];
    WbDeviceTag sensor_tags[JOINT_NUM];
    for (int i = 0; i < JOINT_NUM; i++)
    {
        motor_tags[i] = wb_robot_get_device(motor_names[i]);
        wb_motor_set_position(motor_tags[i], 0);
        //wb_motor_set_velocity(motor_tags[i], 100.0); // rad/s

        sensor_tags[i] = wb_robot_get_device(sensor_names[i]);
        wb_position_sensor_enable(sensor_tags[i], TIME_STEP);
    }
    joint_s joints[JOINT_NUM] = {0};
    double step = 0;

    // arm_matrix_instance_f32 test_mat_0 = {0};
    // float *test_mat_0_data = (float *)malloc(sizeof(float) * 9);
    // memset(test_mat_0_data, 0, sizeof(float) * 9);
    // arm_mat_init_f32(&test_mat_0, 3, 3, test_mat_0_data);
    // for (int i = 0; i < 9; i++)
    //     test_mat_0_data[i] = 1 + i;

    // float test_eulars[3] = {pi / 3, pi / 4, pi / 5};
    // printf("%f\t%f\t%f\r\n", test_eulars[0], test_eulars[1], test_eulars[2]);
    // EularToRotmat(test_eulars[0], test_eulars[1], test_eulars[2], &test_mat_0);
    // printMat(&test_mat_0);

    // RotmatToEular(&test_mat_0, test_eulars);
    // printf("%f\t%f\t%f\r\n", test_eulars[0], test_eulars[1], test_eulars[2]);

    int pressed_key = 0;
    wb_keyboard_enable(TIME_STEP);

    float set_pos6[6] = {0};
    set_pos6[0] = 0.56f;
    set_pos6[1] = 0.0f;
    set_pos6[2] = 0.0f;
    set_pos6[3] = 0.0f;
    set_pos6[4] = 0.0f;
    set_pos6[5] = 0.0f;

    float pos6[6] = {0};
    float eular_angle[3] = {0};

    float d_1 = 0.0f;
    float theta_2 = 0.0f;
    float theta_3 = 0.0f;
    float theta_4 = 0.0f;
    float theta_5 = 0.0f;
    float theta_6 = 0.0f;

    rfl_matrix_instance robot = {0};
    float robot_data[16] = {0};
    rflMatrixInit(&robot, 4, 4, robot_data);

    rfl_matrix_instance t_[10] = {0};
    float t_data[10][16] = {0};
    for (int i = 0; i < 10; i++)
    {
        rflMatrixInit(&t_[i], 4, 4, t_data[i]);
        // printMat(&t_[i]);
    }

    t_data[0][0] = 0;
    t_data[0][1] = 0;
    t_data[0][2] = 1;
    t_data[0][3] = 0;
    t_data[0][4] = 0;
    t_data[0][5] = -1;
    t_data[0][6] = 0;
    t_data[0][7] = 0;
    t_data[0][8] = 1;
    t_data[0][9] = 0;
    t_data[0][10] = 0;
    t_data[0][11] = 0;
    t_data[0][12] = 0;
    t_data[0][13] = 0;
    t_data[0][14] = 0;
    t_data[0][15] = 1;
    DhToTransmat(&t_[1], 0, 0, d_1, 0);
    DhToTransmat(&t_[2], 0, 0, 0, theta_2);
    DhToTransmat(&t_[3], 0, L1, 0, theta_3);
    DhToTransmat(&t_[4], 0, L2, 0, theta_4);
    DhToTransmat(&t_[5], pi / 2, L3, 0, theta_5 - pi / 2);
    DhToTransmat(&t_[6], -pi / 2, 0, L4, theta_6);
    t_data[7][0] = -1;
    t_data[7][1] = 0;
    t_data[7][2] = 0;
    t_data[7][3] = 0;
    t_data[7][4] = 0;
    t_data[7][5] = -1;
    t_data[7][6] = 0;
    t_data[7][7] = 0;
    t_data[7][8] = 0;
    t_data[7][9] = 0;
    t_data[7][10] = 1;
    t_data[7][11] = 0;
    t_data[7][12] = 0;
    t_data[7][13] = 0;
    t_data[7][14] = 0;
    t_data[7][15] = 1;

    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    while (wb_robot_step(TIME_STEP) != -1)
    {
        /*
         * Read the sensors :
         * Enter here functions to read sensor data, like:
         *  double val = wb_distance_sensor_get_value(my_sensor);
         */
        for (int i = 0; i < JOINT_NUM; i++)
        {
            joints[i].now = wb_position_sensor_get_value(sensor_tags[i]);
        }

        DhToTransmat(&t_[1], 0, 0, joints[0].now, 0);
        DhToTransmat(&t_[2], 0, 0, 0, joints[1].now);
        DhToTransmat(&t_[3], 0, L1, 0, joints[2].now);
        DhToTransmat(&t_[4], 0, L2, 0, joints[3].now);
        DhToTransmat(&t_[5], pi / 2, L3, 0, joints[4].now - pi / 2);
        DhToTransmat(&t_[6], -pi / 2, 0, L4, joints[5].now);

        // rflMatrixMult(&t_[0], &t_[1], &t_[8]);
        // rflMatrixMult(&t_[8], &t_[2], &t_[9]);

        rflMatrixMult(&t_[1], &t_[2], &t_[9]);

        rflMatrixMult(&t_[9], &t_[3], &t_[8]);
        rflMatrixMult(&t_[8], &t_[4], &t_[9]);
        rflMatrixMult(&t_[9], &t_[5], &t_[8]);
        rflMatrixMult(&t_[8], &t_[6], &t_[9]);
        rflMatrixMult(&t_[9], &t_[7], &robot);

        printMat(&robot);

        pos6[0] = robot.pData[3];
        pos6[1] = robot.pData[7];
        pos6[2] = robot.pData[11];
        pos6[3] = joints[1].now + joints[2].now + joints[3].now;
        pos6[4] = joints[4].now;
        pos6[5] = joints[5].now;
        printf("set_x    :%8.4f\tset_y    :%8.4f\tset_z    :%8.4f\r\nset_yaw  :%8.4f\tset_pitch:%8.4f\tset_roll "
               ":%8.4f\r\n",
               set_pos6[0], set_pos6[1], set_pos6[2], set_pos6[5], set_pos6[4], set_pos6[3]);
        printf("now_x    :%8.4f\tnow_y    :%8.4f\tnow_z    :%8.4f\r\nnow_yaw  :%8.4f\tnow_pitch:%8.4f\tnow_roll "
               ":%8.4f\r\n",
               pos6[0], pos6[1], pos6[2], pos6[3], pos6[4], pos6[5]);

        printf("joint_set:%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t\r\n", joints[0].set, joints[1].set, joints[2].set,
               joints[3].set, joints[4].set, joints[5].set);
        printf("joint_not:%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t\r\n", joints[0].now, joints[1].now, joints[2].now,
               joints[3].now, joints[4].now, joints[5].now);

        pressed_key = wb_keyboard_get_key();
     
        if (pressed_key == 'W')
        {
            set_pos6[0] += 0.001;
        }
        else if (pressed_key == 'S')
        {
            set_pos6[0] -= 0.001;
        }
        else if (pressed_key == 'A')
        {
            set_pos6[1] += 0.001;
        }
        else if (pressed_key == 'D')
        {
            set_pos6[1] -= 0.001;
        }
        else if (pressed_key == 'C')
        {
            set_pos6[2] += 0.001;
        }
        else if (pressed_key == 'X')
        {
            set_pos6[2] -= 0.001;
        }
        else if (pressed_key == 'U')
        {
            set_pos6[3] += 0.001;
        }
        else if (pressed_key == 'O')
        {
            set_pos6[3] -= 0.001;
        }
        else if (pressed_key == 'I')
        {
            set_pos6[4] += 0.001;
        }
        else if (pressed_key == 'K')
        {
            set_pos6[4] -= 0.001;
        }
        else if (pressed_key == 'J')
        {
            set_pos6[5] += 0.001;
        }
        else if (pressed_key == 'L')
        {
            set_pos6[5] -= 0.001;
        }

//////////////////////////////////////////////////////////////////////////////////
        if (hit_mode)//新增
{
    // ===== 击球动作组模式 =====
    for (int i = 0; i < JOINT_NUM; i++)
    {
        joints[i].set = pick_action[pick_pose_id][i];
    }

    pick_timer++;

    // 当前 pose 走完一段时间后，切下一个
    if (pick_timer > PICK_HOLD_TIME)
    {
        pick_timer = 0;
        pick_pose_id++;

        if (pick_pose_id >= PICK_POSE_NUM)
            pick_pose_id = PICK_POSE_NUM - 1;  // 停在最后
    }
}
else
{
    // ===== 原来的键盘 + IK 模式 =====
    i_kin(set_pos6, joints, pressed_key);
}
/////////////////////////////////////////////////////////////////////////////


        for (int i = 0; i < JOINT_NUM; i++)//关节插值，使动作平滑,但是会限制角速度，让动作变慢。
        {
            if (i == 4)//控制5号关节，让它“步幅”变大，还不影响其他关节，防止其他位置产生抖动。
            {
                step = 0.8;
                if (fabs(joints[i].now - joints[i].set) > 0.8)//fabs:取浮点数的绝对值
                {
                    if (joints[i].now < joints[i].set)
                        joints[i].out = joints[i].now + step;
                    else if (joints[i].now > joints[i].set)
                        joints[i].out = joints[i].now - step;
                }
            }
            else
            {
                step = 0.01;
                if (fabs(joints[i].now - joints[i].set) > 0.01)
                {
                    if (joints[i].now < joints[i].set)
                        joints[i].out = joints[i].now + step;
                    else if (joints[i].now > joints[i].set)
                        joints[i].out = joints[i].now - step;
                }
            }
            // joints[i].out = joints[i].set;
        }

        for (int i = 0; i < JOINT_NUM; i++)
        {
            if (joints[i].out > wb_motor_get_max_position(motor_tags[i]))
                joints[i].out = wb_motor_get_max_position(motor_tags[i]);
            else if (joints[i].out < wb_motor_get_min_position(motor_tags[i]))
                joints[i].out = wb_motor_get_min_position(motor_tags[i]);
        }

        /* Process sensor data here */

        /*
         * Enter here functions to send actuator commands, like:
         * wb_motor_set_position(my_actuator, 10.0);
         */
        for (int i = 0; i < JOINT_NUM; i++)
        {
            wb_motor_set_position(motor_tags[i], joints[i].out);
        }
    };

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}

void rflMatrixInit(rfl_matrix_instance *S, uint16_t nRows, uint16_t nColumns, float *pData)
{
    /* 分配行数 */
    S->numRows = nRows;
    /* 分配列数 */
    S->numCols = nColumns;
    /* 分配数据指针 */
    S->pData = pData;
}

rfl_matrix_status rflMatrixAdd(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst)
{
    float *pIn1 = pSrcA->pData; /* input data matrix pointer A  */
    float *pIn2 = pSrcB->pData; /* input data matrix pointer B  */
    float *pOut = pDst->pData;  /* output data matrix pointer   */

    float inA1, inA2, inB1, inB2, out1, out2; /* temporary variables */

    uint32_t numSamples;      /* total number of elements in the matrix  */
    uint32_t blkCnt;          /* loop counters */
    rfl_matrix_status status; /* status of matrix addition */

    {

        /* Total number of samples in the input matrix */
        numSamples = (uint32_t)pSrcA->numRows * pSrcA->numCols;

        /* Loop unrolling */
        blkCnt = numSamples >> 2u;

        /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
         ** a second loop below computes the remaining 1 to 3 samples. */
        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) + B(m,n) */
            /* Add and then store the results in the destination buffer. */
            /* Read values from source A */
            inA1 = pIn1[0];

            /* Read values from source B */
            inB1 = pIn2[0];

            /* Read values from source A */
            inA2 = pIn1[1];

            /* out = sourceA + sourceB */
            out1 = inA1 + inB1;

            /* Read values from source B */
            inB2 = pIn2[1];

            /* Read values from source A */
            inA1 = pIn1[2];

            /* out = sourceA + sourceB */
            out2 = inA2 + inB2;

            /* Read values from source B */
            inB1 = pIn2[2];

            /* Store result in destination */
            pOut[0] = out1;
            pOut[1] = out2;

            /* Read values from source A */
            inA2 = pIn1[3];

            /* Read values from source B */
            inB2 = pIn2[3];

            /* out = sourceA + sourceB */
            out1 = inA1 + inB1;

            /* out = sourceA + sourceB */
            out2 = inA2 + inB2;

            /* Store result in destination */
            pOut[2] = out1;

            /* Store result in destination */
            pOut[3] = out2;

            /* update pointers to process next sampels */
            pIn1 += 4u;
            pIn2 += 4u;
            pOut += 4u;
            /* Decrement the loop counter */
            blkCnt--;
        }

        /* If the numSamples is not a multiple of 4, compute any remaining output samples here.
         ** No loop unrolling is used. */
        blkCnt = numSamples % 0x4u;

        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) + B(m,n) */
            /* Add and then store the results in the destination buffer. */
            *pOut++ = (*pIn1++) + (*pIn2++);

            /* Decrement the loop counter */
            blkCnt--;
        }

        /* set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixSub(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst)
{
    float *pIn1 = pSrcA->pData; /* input data matrix pointer A */
    float *pIn2 = pSrcB->pData; /* input data matrix pointer B */
    float *pOut = pDst->pData;  /* output data matrix pointer  */

    float inA1, inA2, inB1, inB2, out1, out2; /* temporary variables */

    uint32_t numSamples;      /* total number of elements in the matrix  */
    uint32_t blkCnt;          /* loop counters */
    rfl_matrix_status status; /* status of matrix subtraction */

    {
        /* Total number of samples in the input matrix */
        numSamples = (uint32_t)pSrcA->numRows * pSrcA->numCols;

        /* Run the below code for Cortex-M4 and Cortex-M3 */

        /* Loop Unrolling */
        blkCnt = numSamples >> 2u;

        /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
         ** a second loop below computes the remaining 1 to 3 samples. */
        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) - B(m,n) */
            /* Subtract and then store the results in the destination buffer. */
            /* Read values from source A */
            inA1 = pIn1[0];

            /* Read values from source B */
            inB1 = pIn2[0];

            /* Read values from source A */
            inA2 = pIn1[1];

            /* out = sourceA - sourceB */
            out1 = inA1 - inB1;

            /* Read values from source B */
            inB2 = pIn2[1];

            /* Read values from source A */
            inA1 = pIn1[2];

            /* out = sourceA - sourceB */
            out2 = inA2 - inB2;

            /* Read values from source B */
            inB1 = pIn2[2];

            /* Store result in destination */
            pOut[0] = out1;
            pOut[1] = out2;

            /* Read values from source A */
            inA2 = pIn1[3];

            /* Read values from source B */
            inB2 = pIn2[3];

            /* out = sourceA - sourceB */
            out1 = inA1 - inB1;

            /* out = sourceA - sourceB */
            out2 = inA2 - inB2;

            /* Store result in destination */
            pOut[2] = out1;

            /* Store result in destination */
            pOut[3] = out2;

            /* update pointers to process next sampels */
            pIn1 += 4u;
            pIn2 += 4u;
            pOut += 4u;

            /* Decrement the loop counter */
            blkCnt--;
        }

        /* If the numSamples is not a multiple of 4, compute any remaining output samples here.
         ** No loop unrolling is used. */
        blkCnt = numSamples % 0x4u;

        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) - B(m,n) */
            /* Subtract and then store the results in the destination buffer. */
            *pOut++ = (*pIn1++) - (*pIn2++);

            /* Decrement the loop counter */
            blkCnt--;
        }

        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixMult(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                                rfl_matrix_instance *pDst)
{
    float *pIn1 = pSrcA->pData;         /* input data matrix pointer A */
    float *pIn2 = pSrcB->pData;         /* input data matrix pointer B */
    float *pInA = pSrcA->pData;         /* input data matrix pointer A  */
    float *pOut = pDst->pData;          /* output data matrix pointer */
    float *px;                          /* Temporary output data matrix pointer */
    float sum;                          /* Accumulator */
    uint16_t numRowsA = pSrcA->numRows; /* number of rows of input matrix A */
    uint16_t numColsB = pSrcB->numCols; /* number of columns of input matrix B */
    uint16_t numColsA = pSrcA->numCols; /* number of columns of input matrix A */

    /* Run the below code for Cortex-M4 and Cortex-M3 */

    float in1, in2, in3, in4;
    uint16_t col, i = 0u, j, row = numRowsA, colCnt; /* loop counters */
    rfl_matrix_status status;                        /* status of matrix multiplication */

    {
        /* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
        /* row loop */
        do
        {
            /* Output pointer is set to starting address of the row being processed */
            px = pOut + i;

            /* For every row wise process, the column loop counter is to be initiated */
            col = numColsB;

            /* For every row wise process, the pIn2 pointer is set
             ** to the starting address of the pSrcB data */
            pIn2 = pSrcB->pData;

            j = 0u;

            /* column loop */
            do
            {
                /* Set the variable sum, that acts as accumulator, to zero */
                sum = 0.0f;

                /* Initiate the pointer pIn1 to point to the starting address of the column being processed */
                pIn1 = pInA;

                /* Apply loop unrolling and compute 4 MACs simultaneously. */
                colCnt = numColsA >> 2u;

                /* matrix multiplication        */
                while (colCnt > 0u)
                {
                    /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
                    in3 = *pIn2;
                    pIn2 += numColsB;
                    in1 = pIn1[0];
                    in2 = pIn1[1];
                    sum += in1 * in3;
                    in4 = *pIn2;
                    pIn2 += numColsB;
                    sum += in2 * in4;

                    in3 = *pIn2;
                    pIn2 += numColsB;
                    in1 = pIn1[2];
                    in2 = pIn1[3];
                    sum += in1 * in3;
                    in4 = *pIn2;
                    pIn2 += numColsB;
                    sum += in2 * in4;
                    pIn1 += 4u;

                    /* Decrement the loop count */
                    colCnt--;
                }

                /* If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.
                 ** No loop unrolling is used. */
                colCnt = numColsA % 0x4u;

                while (colCnt > 0u)
                {
                    /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
                    sum += *pIn1++ * (*pIn2);
                    pIn2 += numColsB;

                    /* Decrement the loop counter */
                    colCnt--;
                }

                /* Store the result in the destination buffer */
                *px++ = sum;

                /* Update the pointer pIn2 to point to the  starting address of the next column */
                j++;
                pIn2 = pSrcB->pData + j;

                /* Decrement the column loop counter */
                col--;

            } while (col > 0u);

            /* Update the pointer pInA to point to the  starting address of the next row */
            i = i + numColsB;
            pInA = pInA + numColsA;

            /* Decrement the row loop counter */
            row--;

        } while (row > 0u);
        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixTrans(const rfl_matrix_instance *pSrc, rfl_matrix_instance *pDst)
{
    float *pIn = pSrc->pData;          /* input data matrix pointer */
    float *pOut = pDst->pData;         /* output data matrix pointer */
    float *px;                         /* Temporary output data matrix pointer */
    uint16_t nRows = pSrc->numRows;    /* number of rows */
    uint16_t nColumns = pSrc->numCols; /* number of columns */

    /* Run the below code for Cortex-M4 and Cortex-M3 */

    uint16_t blkCnt, i = 0u, row = nRows; /* loop counters */
    rfl_matrix_status status;             /* status of matrix transpose  */

    {
        /* Matrix transpose by exchanging the rows with columns */
        /* row loop     */
        do
        {
            /* Loop Unrolling */
            blkCnt = nColumns >> 2;

            /* The pointer px is set to starting address of the column being processed */
            px = pOut + i;

            /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
             ** a second loop below computes the remaining 1 to 3 samples. */
            while (blkCnt > 0u) /* column loop */
            {
                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Decrement the column loop counter */
                blkCnt--;
            }

            /* Perform matrix transpose for last 3 samples here. */
            blkCnt = nColumns % 0x4u;

            while (blkCnt > 0u)
            {
                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Decrement the column loop counter */
                blkCnt--;
            }

            i++;

            /* Decrement the row loop counter */
            row--;

        } while (row > 0u); /* row loop end  */

        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixInverse(const rfl_matrix_instance *pSrc, rfl_matrix_instance *pDst)
{
    float *pIn = pSrc->pData;                                /* input data matrix pointer */
    float *pOut = pDst->pData;                               /* output data matrix pointer */
    float *pInT1, *pInT2;                                    /* Temporary input data matrix pointer */
    float *pOutT1, *pOutT2;                                  /* Temporary output data matrix pointer */
    float *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst; /* Temporary input and output data matrix pointer */
    uint32_t numRows = pSrc->numRows;                        /* Number of rows in the matrix  */
    uint32_t numCols = pSrc->numCols;                        /* Number of Cols in the matrix  */

    float maxC; /* maximum value in the column */

    /* Run the below code for Cortex-M4 and Cortex-M3 */

    float Xchg, in = 0.0f, in1;                      /* Temporary input values  */
    uint32_t i, rowCnt, flag = 0u, j, loopCnt, k, l; /* loop counters */
    rfl_matrix_status status;                        /* status of matrix inverse */

    {

        /*--------------------------------------------------------------------------------------------------------------
         * Matrix Inverse can be solved using elementary row operations.
         *
         *	Gauss-Jordan Method:
         *
         *	   1. First combine the identity matrix and the input matrix separated by a bar to form an
         *        augmented matrix as follows:
         *				        _ 	      	       _         _	       _
         *					   |  a11  a12 | 1   0  |       |  X11 X12  |
         *					   |           |        |   =   |           |
         *					   |_ a21  a22 | 0   1 _|       |_ X21 X21 _|
         *
         *		2. In our implementation, pDst Matrix is used as identity matrix.
         *
         *		3. Begin with the first row. Let i = 1.
         *
         *	    4. Check to see if the pivot for column i is the greatest of the column.
         *		   The pivot is the element of the main diagonal that is on the current row.
         *		   For instance, if working with row i, then the pivot element is aii.
         *		   If the pivot is not the most significant of the columns, exchange that row with a row
         *		   below it that does contain the most significant value in column i. If the most
         *         significant value of the column is zero, then an inverse to that matrix does not exist.
         *		   The most significant value of the column is the absolute maximum.
         *
         *	    5. Divide every element of row i by the pivot.
         *
         *	    6. For every row below and  row i, replace that row with the sum of that row and
         *		   a multiple of row i so that each new element in column i below row i is zero.
         *
         *	    7. Move to the next row and column and repeat steps 2 through 5 until you have zeros
         *		   for every element below and above the main diagonal.
         *
         *		8. Now an identical matrix is formed to the left of the bar(input matrix, pSrc).
         *		   Therefore, the matrix to the right of the bar is our solution(pDst matrix, pDst).
         *----------------------------------------------------------------------------------------------------------------*/

        /* Working pointer for destination matrix */
        pOutT1 = pOut;

        /* Loop over the number of rows */
        rowCnt = numRows;

        /* Making the destination matrix as identity matrix */
        while (rowCnt > 0u)
        {
            /* Writing all zeroes in lower triangle of the destination matrix */
            j = numRows - rowCnt;
            while (j > 0u)
            {
                *pOutT1++ = 0.0f;
                j--;
            }

            /* Writing all ones in the diagonal of the destination matrix */
            *pOutT1++ = 1.0f;

            /* Writing all zeroes in upper triangle of the destination matrix */
            j = rowCnt - 1u;
            while (j > 0u)
            {
                *pOutT1++ = 0.0f;
                j--;
            }

            /* Decrement the loop counter */
            rowCnt--;
        }

        /* Loop over the number of columns of the input matrix.
           All the elements in each column are processed by the row operations */
        loopCnt = numCols;

        /* Index modifier to navigate through the columns */
        l = 0u;

        while (loopCnt > 0u)
        {
            /* Check if the pivot element is zero..
             * If it is zero then interchange the row with non zero row below.
             * If there is no non zero element to replace in the rows below,
             * then the matrix is Singular. */

            /* Working pointer for the input matrix that points
             * to the pivot element of the particular row  */
            pInT1 = pIn + (l * numCols);

            /* Working pointer for the destination matrix that points
             * to the pivot element of the particular row  */
            pOutT1 = pOut + (l * numCols);

            /* Temporary variable to hold the pivot value */
            in = *pInT1;

            /* Grab the most significant value from column l */
            maxC = 0;
            for (i = l; i < numRows; i++)
            {
                maxC = *pInT1 > 0 ? (*pInT1 > maxC ? *pInT1 : maxC) : (-*pInT1 > maxC ? -*pInT1 : maxC);
                pInT1 += numCols;
            }

            /* Update the status if the matrix is singular */
            if (maxC == 0.0f)
            {
                return RFL_MATRIX_SINGULAR;
            }

            /* Restore pInT1  */
            pInT1 = pIn;

            /* Destination pointer modifier */
            k = 1u;

            /* Check if the pivot element is the most significant of the column */
            if ((in > 0.0f ? in : -in) != maxC)
            {
                /* Loop over the number rows present below */
                i = numRows - (l + 1u);

                while (i > 0u)
                {
                    /* Update the input and destination pointers */
                    pInT2 = pInT1 + (numCols * l);
                    pOutT2 = pOutT1 + (numCols * k);

                    /* Look for the most significant element to
                     * replace in the rows below */
                    if ((*pInT2 > 0.0f ? *pInT2 : -*pInT2) == maxC)
                    {
                        /* Loop over number of columns
                         * to the right of the pilot element */
                        j = numCols - l;

                        while (j > 0u)
                        {
                            /* Exchange the row elements of the input matrix */
                            Xchg = *pInT2;
                            *pInT2++ = *pInT1;
                            *pInT1++ = Xchg;

                            /* Decrement the loop counter */
                            j--;
                        }

                        /* Loop over number of columns of the destination matrix */
                        j = numCols;

                        while (j > 0u)
                        {
                            /* Exchange the row elements of the destination matrix */
                            Xchg = *pOutT2;
                            *pOutT2++ = *pOutT1;
                            *pOutT1++ = Xchg;

                            /* Decrement the loop counter */
                            j--;
                        }

                        /* Flag to indicate whether exchange is done or not */
                        flag = 1u;

                        /* Break after exchange is done */
                        break;
                    }

                    /* Update the destination pointer modifier */
                    k++;

                    /* Decrement the loop counter */
                    i--;
                }
            }

            /* Update the status if the matrix is singular */
            if ((flag != 1u) && (in == 0.0f))
            {
                return RFL_MATRIX_SINGULAR;
            }

            /* Points to the pivot row of input and destination matrices */
            pPivotRowIn = pIn + (l * numCols);
            pPivotRowDst = pOut + (l * numCols);

            /* Temporary pointers to the pivot row pointers */
            pInT1 = pPivotRowIn;
            pInT2 = pPivotRowDst;

            /* Pivot element of the row */
            in = *pPivotRowIn;

            /* Loop over number of columns
             * to the right of the pilot element */
            j = (numCols - l);

            while (j > 0u)
            {
                /* Divide each element of the row of the input matrix
                 * by the pivot element */
                in1 = *pInT1;
                *pInT1++ = in1 / in;

                /* Decrement the loop counter */
                j--;
            }

            /* Loop over number of columns of the destination matrix */
            j = numCols;

            while (j > 0u)
            {
                /* Divide each element of the row of the destination matrix
                 * by the pivot element */
                in1 = *pInT2;
                *pInT2++ = in1 / in;

                /* Decrement the loop counter */
                j--;
            }

            /* Replace the rows with the sum of that row and a multiple of row i
             * so that each new element in column i above row i is zero.*/

            /* Temporary pointers for input and destination matrices */
            pInT1 = pIn;
            pInT2 = pOut;

            /* index used to check for pivot element */
            i = 0u;

            /* Loop over number of rows */
            /*  to be replaced by the sum of that row and a multiple of row i */
            k = numRows;

            while (k > 0u)
            {
                /* Check for the pivot element */
                if (i == l)
                {
                    /* If the processing element is the pivot element,
                       only the columns to the right are to be processed */
                    pInT1 += numCols - l;

                    pInT2 += numCols;
                }
                else
                {
                    /* Element of the reference row */
                    in = *pInT1;

                    /* Working pointers for input and destination pivot rows */
                    pPRT_in = pPivotRowIn;
                    pPRT_pDst = pPivotRowDst;

                    /* Loop over the number of columns to the right of the pivot element,
                       to replace the elements in the input matrix */
                    j = (numCols - l);

                    while (j > 0u)
                    {
                        /* Replace the element by the sum of that row
                           and a multiple of the reference row  */
                        in1 = *pInT1;
                        *pInT1++ = in1 - (in * *pPRT_in++);

                        /* Decrement the loop counter */
                        j--;
                    }

                    /* Loop over the number of columns to
                       replace the elements in the destination matrix */
                    j = numCols;

                    while (j > 0u)
                    {
                        /* Replace the element by the sum of that row
                           and a multiple of the reference row  */
                        in1 = *pInT2;
                        *pInT2++ = in1 - (in * *pPRT_pDst++);

                        /* Decrement the loop counter */
                        j--;
                    }
                }

                /* Increment the temporary input pointer */
                pInT1 = pInT1 + l;

                /* Decrement the loop counter */
                k--;

                /* Increment the pivot index */
                i++;
            }

            /* Increment the input pointer */
            pIn++;

            /* Decrement the loop counter */
            loopCnt--;

            /* Increment the index modifier */
            l++;
        }

        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;

        if ((flag != 1u) && (in == 0.0f))
        {
            pIn = pSrc->pData;
            for (i = 0; i < numRows * numCols; i++)
            {
                if (pIn[i] != 0.0f)
                    break;
            }

            if (i == numRows * numCols)
                status = RFL_MATRIX_SINGULAR;
        }
    }
    /* Return to application */
    return (status);
}
