#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rm_referee_interfaces/msg/decision.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class RefereeExecutor : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  RefereeExecutor() : Node("referee_executor")
  {
    decision_sub_ = this->create_subscription<rm_referee_interfaces::msg::Decision>(
      "/decision", 10,
      std::bind(&RefereeExecutor::decision_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
    cmd_vel_raw_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&RefereeExecutor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "裁判执行节点启动");
  }

private:
  void decision_callback(const rm_referee_interfaces::msg::Decision::SharedPtr msg)
  {
    using D = rm_referee_interfaces::msg::Decision;

    if (msg->decision == current_decision_)
      return;

    current_decision_ = msg->decision;

    if (msg->decision == D::IDLE || msg->decision == D::SHUTDOWN)
    {
      nav_action_client_->async_cancel_all_goals();
      RCLCPP_INFO(this->get_logger(), "IDLE/SHUTDOWN: 取消所有导航目标");
    }
    else if (msg->decision == D::RETREAT)
    {
      send_retreat_goal();
    }
    else if (msg->decision != D::RETREAT && retreat_active_)
    {
      nav_action_client_->async_cancel_all_goals();
      retreat_active_ = false;
      RCLCPP_INFO(this->get_logger(), "取消回原点");
    }
  }

  void send_retreat_goal()
  {
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(this->get_logger(), "Nav2 动作服务器未就绪，无法回原点");
      return;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = 0.0;
    goal.pose.pose.position.y = 0.0;
    goal.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&RefereeExecutor::retreat_result_callback, this, std::placeholders::_1);

    nav_action_client_->async_send_goal(goal, send_goal_options);
    retreat_active_ = true;
    RCLCPP_INFO(this->get_logger(), "RETREAT: 导航回原点 (0, 0)");
  }

  void retreat_result_callback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    retreat_active_ = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      RCLCPP_INFO(this->get_logger(), "已到达原点");
    else
      RCLCPP_WARN(this->get_logger(), "回原点失败");
  }

  void timer_callback()
  {
    using D = rm_referee_interfaces::msg::Decision;

    if (current_decision_ == D::IDLE || current_decision_ == D::SHUTDOWN)
    {
      auto stop = geometry_msgs::msg::Twist();
      cmd_vel_pub_->publish(stop);
      cmd_vel_raw_pub_->publish(stop);
    }
  }

  uint8_t current_decision_ = rm_referee_interfaces::msg::Decision::ENGAGE;
  bool retreat_active_ = false;

  rclcpp::Subscription<rm_referee_interfaces::msg::Decision>::SharedPtr decision_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RefereeExecutor>());
  rclcpp::shutdown();
  return 0;
}
