#include <rclcpp/rclcpp.hpp>
#include <rm_referee_interfaces/msg/game_status.hpp>
#include <rm_referee_interfaces/msg/robot_status.hpp>
#include <rm_referee_interfaces/msg/decision.hpp>

class DecisionNode : public rclcpp::Node
{
public:
  DecisionNode() : Node("decision_node")
  {
    this->declare_parameter<double>("hp_retreat_ratio", 0.2);
    this->declare_parameter<double>("hp_cautious_ratio", 0.4);
    this->declare_parameter<double>("ammo_low_ratio", 0.15);
    this->declare_parameter<double>("heat_cool_ratio", 0.8);
    this->declare_parameter<double>("heat_conservative_ratio", 0.5);
    this->declare_parameter<double>("ammo_conservative_ratio", 0.4);

    hp_retreat_ = this->get_parameter("hp_retreat_ratio").as_double();
    hp_cautious_ = this->get_parameter("hp_cautious_ratio").as_double();
    ammo_low_ = this->get_parameter("ammo_low_ratio").as_double();
    heat_cool_ = this->get_parameter("heat_cool_ratio").as_double();
    heat_conservative_ = this->get_parameter("heat_conservative_ratio").as_double();
    ammo_conservative_ = this->get_parameter("ammo_conservative_ratio").as_double();

    current_stage_ = rm_referee_interfaces::msg::GameStatus::STAGE_PREPARATION;

    game_status_sub_ = this->create_subscription<rm_referee_interfaces::msg::GameStatus>(
      "/referee/game_status", 10,
      std::bind(&DecisionNode::game_status_callback, this, std::placeholders::_1));

    robot_status_sub_ = this->create_subscription<rm_referee_interfaces::msg::RobotStatus>(
      "/referee/robot_status", 10,
      std::bind(&DecisionNode::robot_status_callback, this, std::placeholders::_1));

    decision_pub_ = this->create_publisher<rm_referee_interfaces::msg::Decision>(
      "/decision", 10);

    RCLCPP_INFO(this->get_logger(), "决策节点启动");
  }

private:
  void game_status_callback(const rm_referee_interfaces::msg::GameStatus::SharedPtr msg)
  {
    uint8_t old_stage = current_stage_;
    current_stage_ = msg->stage;

    if (msg->stage != old_stage)
    {
      evaluate();
    }
  }

  void robot_status_callback(const rm_referee_interfaces::msg::RobotStatus::SharedPtr msg)
  {
    last_robot_status_ = *msg;
    evaluate();
  }

  void evaluate()
  {
    using GS = rm_referee_interfaces::msg::GameStatus;
    auto d = rm_referee_interfaces::msg::Decision();

    switch (current_stage_)
    {
      case GS::STAGE_PREPARATION:
        d.decision = d.IDLE;
        d.description = "准备阶段，原地待命";
        break;

      case GS::STAGE_SELF_CHECK:
        d.decision = d.IDLE;
        d.description = "自检阶段，等待初始化";
        break;

      case GS::STAGE_RUNNING:
        d = evaluate_running();
        break;

      case GS::STAGE_GAME_OVER:
        d.decision = d.SHUTDOWN;
        d.description = "比赛结束，关机";
        break;
    }

    publish_decision(d);
  }

  rm_referee_interfaces::msg::Decision evaluate_running()
  {
    auto d = rm_referee_interfaces::msg::Decision();

    double hp_ratio   = static_cast<double>(last_robot_status_.remain_hp) / last_robot_status_.max_hp;
    double ammo_ratio = (last_robot_status_.ammo_count > 0) ? 1.0 : 0.0;
    double heat_ratio = static_cast<double>(last_robot_status_.heat) / last_robot_status_.heat_limit;

    if (hp_ratio <= hp_retreat_)
    {
      d.decision = d.RETREAT;
      d.description = "血量过低(" + std::to_string(static_cast<int>(hp_ratio * 100)) + "%)，回补给点";
    }
    else if (ammo_ratio <= ammo_low_)
    {
      d.decision = d.RETREAT;
      d.description = "弹丸不足，回补给点";
    }
    else if (heat_ratio >= heat_cool_)
    {
      d.decision = d.COOLDOWN;
      d.description = "枪口过热(" + std::to_string(static_cast<int>(heat_ratio * 100)) + "%)，强制散热";
    }
    else if (hp_ratio <= hp_cautious_)
    {
      d.decision = d.CAUTIOUS;
      d.description = "血量偏低(" + std::to_string(static_cast<int>(hp_ratio * 100)) + "%)，保守防守";
    }
    else if (heat_ratio >= heat_conservative_ && ammo_ratio <= ammo_conservative_)
    {
      d.decision = d.PATROL;
      d.description = "热量偏高且弹药偏低，巡航待机";
    }
    else if (heat_ratio >= heat_conservative_)
    {
      d.decision = d.CAUTIOUS;
      d.description = "热量偏高(" + std::to_string(static_cast<int>(heat_ratio * 100)) + "%)，减少射击";
    }
    else
    {
      d.decision = d.ENGAGE;
      d.description = "主动进攻";
    }

    return d;
  }

  void publish_decision(const rm_referee_interfaces::msg::Decision & d)
  {
    static uint8_t last_decision = 99;
    if (d.decision != last_decision)
    {
      last_decision = d.decision;
      RCLCPP_INFO(this->get_logger(), "决策: %s", d.description.c_str());
    }
    decision_pub_->publish(d);
  }

  uint8_t current_stage_;
  rm_referee_interfaces::msg::RobotStatus last_robot_status_;

  double hp_retreat_, hp_cautious_, ammo_low_;
  double heat_cool_, heat_conservative_, ammo_conservative_;

  rclcpp::Subscription<rm_referee_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<rm_referee_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<rm_referee_interfaces::msg::Decision>::SharedPtr decision_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionNode>());
  rclcpp::shutdown();
  return 0;
}
