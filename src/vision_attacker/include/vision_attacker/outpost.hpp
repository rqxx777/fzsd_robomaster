#ifndef OUT_POST_HPP
#define OUT_POST_HPP

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

// 单个装甲板预测状态
struct armorStates
{
    double x;           // 装甲板在惯性系下的 x 坐标
    double y;           // 装甲板在惯性系下的 y 坐标
    double z;           // 装甲板在惯性系下的 z 坐标
    double delta_yaw;   // 装甲板朝向与自身Yaw的差值
};

// 普通地面机器人装甲板预测模型 (4块装甲板)
class carStates
{
public:
    // 从跟踪器消息更新目标状态
    virtual void update(const auto_aim_interfaces::msg::Target target_msg, const double self_yaw)
    {
        x = target_msg.position.x;
        y = target_msg.position.y;
        z = target_msg.position.z;
        delta_z = target_msg.dz;
        raduis_1 = target_msg.radius_1;
        raduis_2 = target_msg.radius_2;
        yaw = target_msg.yaw;
        v_x = target_msg.velocity.x;
        v_y = target_msg.velocity.y;
        v_z = target_msg.velocity.z;
        v_yaw = target_msg.v_yaw;
        this->self_yaw = self_yaw;
    }
    // 预测 time 秒后所有4块装甲板的位置和朝向
    virtual std::vector<armorStates> getPreArmor(const double time)
    {
        std::vector<armorStates> preArmors;
        for (size_t i = 0; i < 4; i++)
        {
            armorStates armor;
            // 每块装甲板间隔90度 (π/2)
            double armor_yaw = yaw + time * v_yaw + M_PI_2 * i;
            while (armor_yaw < 0) { armor_yaw += 2 * M_PI; }
            while (armor_yaw > 2 * M_PI) { armor_yaw -= 2 * M_PI; }
            // 第1/3块用 radius_1, 第2/4块用 radius_2
            armor.x = x + time * v_x - cos(armor_yaw) * (i % 2 ? raduis_2 : raduis_1);
            armor.y = y + time * v_y - sin(armor_yaw) * (i % 2 ? raduis_2 : raduis_1);
            armor.z = z + time * v_z + (i % 2 ? delta_z : 0.0);
            armor.delta_yaw = armor_yaw - self_yaw;
            armor.delta_yaw = armor.delta_yaw > M_PI ? armor.delta_yaw - M_PI * 2 : armor.delta_yaw;
            preArmors.emplace_back(armor);
        };
        return preArmors;
    }

protected:
    double x, y, z;
    double delta_z;       // 两对装甲板的高度差
    double raduis_1;      // 第1/3装甲板的旋转半径
    double raduis_2;      // 第2/4装甲板的旋转半径
    double yaw;           // 机器人朝向角
    double v_x, v_y, v_z; // 线速度
    double v_yaw;         // 角速度
    double self_yaw;      // 自身Yaw角
};

// 前哨站装甲板预测模型 (3块装甲板，固定旋转)
class outpostStates : public carStates
{
public:
    virtual void update(const auto_aim_interfaces::msg::Target target_msg, const double self_yaw)
    {
        x = target_msg.position.x;
        y = target_msg.position.y;
        z = target_msg.position.z;
        delta_z = 0;
        raduis_1 = 0.553 / 2;  // 前哨站固定半径
        raduis_2 = raduis_1;
        yaw = target_msg.yaw;
        v_x = v_y = v_z = 0;   // 前哨站固定不动
        // 前哨站只判断旋转方向: >0.3 rad/s 为正转，<-0.3 为反转，中间视为静止
        v_yaw = target_msg.v_yaw > 0.3    ? 0.8 * M_PI
                : target_msg.v_yaw < -0.3 ? -0.8 * M_PI
                                          : 0;
        this->self_yaw = self_yaw;
    }
    // 预测 time 秒后所有3块装甲板的位置
    virtual std::vector<armorStates> getPreArmor(const double time)
    {
        std::vector<armorStates> preArmors;
        for (size_t i = 0; i < 3; i++)
        {
            armorStates armor;
            // 每块装甲板间隔120度 (2π/3)
            double armor_yaw = yaw + time * v_yaw + 2 * M_PI / 3 * i;
            while (armor_yaw < 0) { armor_yaw += 2 * M_PI; }
            while (armor_yaw > 2 * M_PI) { armor_yaw -= 2 * M_PI; }
            armor.x = x - cos(armor_yaw) * raduis_1;
            armor.y = y - sin(armor_yaw) * raduis_1;
            armor.z = z;
            armor.delta_yaw = armor_yaw - self_yaw;
            preArmors.emplace_back(armor);
        };
        return preArmors;
    }
};
#endif // OUT_POST_HPP