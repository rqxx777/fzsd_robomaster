#include <rclcpp/rclcpp.hpp>
#include <rm_referee_interfaces/msg/game_status.hpp>
#include <rm_referee_interfaces/msg/robot_status.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <string>

using namespace std::chrono_literals;

class RefereeSimulatorNode : public rclcpp::Node
{
public:
  static constexpr uint8_t STAGE_PREPARATION = 0;
  static constexpr uint8_t STAGE_SELF_CHECK = 1;
  static constexpr uint8_t STAGE_RUNNING = 2;
  static constexpr uint8_t STAGE_GAME_OVER = 3;

  RefereeSimulatorNode() : Node("referee_simulator_node")
  {
    this->declare_parameter<int32_t>("max_hp", 500);
    this->declare_parameter<uint16_t>("max_ammo", 100);
    this->declare_parameter<uint16_t>("heat_limit", 240);
    this->declare_parameter<int32_t>("preparation_time", 60);
    this->declare_parameter<int32_t>("self_check_time", 5);
    this->declare_parameter<int32_t>("running_time", 300);

    max_hp_ = this->get_parameter("max_hp").as_int();
    max_ammo_ = static_cast<uint16_t>(this->get_parameter("max_ammo").as_int());
    heat_limit_ = static_cast<uint16_t>(this->get_parameter("heat_limit").as_int());
    preparation_time_ = this->get_parameter("preparation_time").as_int();
    self_check_time_ = this->get_parameter("self_check_time").as_int();
    running_time_ = this->get_parameter("running_time").as_int();

    stage_ = STAGE_PREPARATION;
    remain_hp_ = max_hp_;
    ammo_count_ = max_ammo_;
    heat_ = 0;
    stage_time_left_ = preparation_time_;

    game_status_pub_ = this->create_publisher<rm_referee_interfaces::msg::GameStatus>(
      "/referee/game_status", 10);
    robot_status_pub_ = this->create_publisher<rm_referee_interfaces::msg::RobotStatus>(
      "/referee/robot_status", 10);

    cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/referee/cmd", 10,
      std::bind(&RefereeSimulatorNode::cmd_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(1s, std::bind(&RefereeSimulatorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "裁判系统模拟器启动");
    RCLCPP_INFO(this->get_logger(), "  血量: %d | 弹丸: %u | 热量上限: %u",
                max_hp_, max_ammo_, heat_limit_);
    RCLCPP_INFO(this->get_logger(), "  当前阶段: PREPARATION (%d 秒)", preparation_time_);
  }

private:
  void timer_callback()
  {
    stage_time_left_--;

    switch (stage_)
    {
      case STAGE_PREPARATION:
      {
        if (stage_time_left_ <= 0)
        {
          switch_to(STAGE_SELF_CHECK);
          stage_time_left_ = self_check_time_;
        }
        break;
      }

      case STAGE_SELF_CHECK:
      {
        remain_hp_ = max_hp_;
        ammo_count_ = max_ammo_;
        heat_ = 0;
        if (stage_time_left_ <= 0)
        {
          switch_to(STAGE_RUNNING);
          stage_time_left_ = running_time_;
        }
        break;
      }

      case STAGE_RUNNING:
      {
        heat_ = static_cast<uint16_t>(std::max(0, static_cast<int>(heat_) - 5));
        if (stage_time_left_ <= 0 || remain_hp_ <= 0)
        {
          switch_to(STAGE_GAME_OVER);
          publish_status();
          RCLCPP_INFO(this->get_logger(), "比赛结束，模拟器退出");
          rclcpp::shutdown();
          return;
        }
        break;
      }

      case STAGE_GAME_OVER:
        publish_status();
        RCLCPP_INFO(this->get_logger(), "比赛结束，模拟器退出");
        rclcpp::shutdown();
        return;
    }

    publish_status();
  }

  void cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    uint8_t cmd = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到指令: %d", cmd);

    switch (cmd)
    {
      case 1:
        switch_to(STAGE_PREPARATION);
        stage_time_left_ = preparation_time_;
        break;
      case 2:
        switch_to(STAGE_SELF_CHECK);
        stage_time_left_ = self_check_time_;
        break;
      case 3:
        switch_to(STAGE_RUNNING);
        stage_time_left_ = running_time_;
        break;
      case 4:
        switch_to(STAGE_GAME_OVER);
        break;
      case 5:
        remain_hp_ = std::max(0, remain_hp_ - 100);
        RCLCPP_INFO(this->get_logger(), "扣血 -100, 当前血量: %d/%d", remain_hp_, max_hp_);
        break;
      case 6:
        ammo_count_ = static_cast<uint16_t>(std::max(0, static_cast<int>(ammo_count_) - 20));
        RCLCPP_INFO(this->get_logger(), "耗弹 -20, 当前弹丸: %u", ammo_count_);
        break;
      case 7:
        heat_ = static_cast<uint16_t>(std::min(static_cast<int>(heat_limit_), static_cast<int>(heat_) + 40));
        RCLCPP_INFO(this->get_logger(), "加热 +40, 当前热量: %u/%u", heat_, heat_limit_);
        break;
      case 8:
        remain_hp_ = max_hp_;
        ammo_count_ = max_ammo_;
        heat_ = 0;
        RCLCPP_INFO(this->get_logger(), "状态重置, HP:%d 弹丸:%u 热量:0", max_hp_, max_ammo_);
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "未知指令: %d", cmd);
        break;
    }
  }

  void switch_to(uint8_t new_stage)
  {
    stage_ = new_stage;

    static const char* stage_names[] = {
      "PREPARATION", "SELF_CHECK", "RUNNING", "GAME_OVER"};

    const char* name = (stage_ <= 3) ? stage_names[stage_] : "UNKNOWN";
    RCLCPP_INFO(this->get_logger(), ">>> 切换到阶段: %s", name);
  }

  void publish_status()
  {
    auto gs = rm_referee_interfaces::msg::GameStatus();
    gs.stage = stage_;
    gs.stage_remain_time = stage_time_left_;
    game_status_pub_->publish(gs);

    auto rs = rm_referee_interfaces::msg::RobotStatus();
    rs.remain_hp = remain_hp_;
    rs.max_hp = max_hp_;
    rs.ammo_count = ammo_count_;
    rs.heat = heat_;
    rs.heat_limit = heat_limit_;
    robot_status_pub_->publish(rs);
  }

  uint8_t stage_;
  int32_t remain_hp_;
  int32_t max_hp_;
  uint16_t ammo_count_;
  uint16_t heat_;
  uint16_t heat_limit_;
  int32_t stage_time_left_;

  int32_t preparation_time_;
  int32_t self_check_time_;
  int32_t running_time_;
  uint16_t max_ammo_;

  rclcpp::Publisher<rm_referee_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<rm_referee_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefereeSimulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
