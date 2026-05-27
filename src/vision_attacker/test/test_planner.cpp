#include <gtest/gtest.h>
#include "vision_attacker/planner.hpp"
#include "vision_attacker/target.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

using namespace std::chrono_literals;

TEST(PlannerTest, BasicPlanFromMsg)
{
  // construct planner with reasonable parameters
  auto_aim::Planner planner(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100.0, 100.0, std::vector<double>{0.1, 0.0}, std::vector<double>{0.1}, std::vector<double>{0.1, 0.0}, std::vector<double>{0.1},
    0.08, 0.04);

  // create a tracker message and fill fields
  auto_aim_interfaces::msg::Target msg;
  msg.tracking = true;
  msg.position.x = 2.0;
  msg.position.y = 0.0;
  msg.position.z = 0.5;
  msg.velocity.x = 0.0;
  msg.velocity.y = 0.0;
  msg.velocity.z = 0.0;
  msg.yaw = 0.0;
  msg.v_yaw = 0.0;
  msg.radius_1 = 0.05;
  msg.radius_2 = 0.05;
  msg.dz = 0.0;
  msg.armors_num = 4;
  msg.id = "car";

  auto_aim::Target t;
  t.from_msg(msg, std::chrono::steady_clock::now());

  auto plan = planner.plan(t, 22.0);

  // Basic expectations: control enabled and angles finite
  EXPECT_TRUE(plan.control);
  EXPECT_FALSE(std::isnan(plan.target_yaw));
  EXPECT_FALSE(std::isnan(plan.target_pitch));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
