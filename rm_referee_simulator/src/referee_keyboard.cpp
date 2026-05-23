#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <termios.h>
#include <unistd.h>
#include <stdio.h>

static char getch()
{
  struct termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = static_cast<char>(getchar());
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("referee_keyboard");
  auto pub = node->create_publisher<std_msgs::msg::UInt8>("/referee/cmd", 10);

  printf("===== 裁判系统键盘控制 =====\n");
  printf("1: 准备阶段 (PREPARATION)\n");
  printf("2: 自检阶段 (SELF_CHECK)\n");
  printf("3: 比赛开始 (RUNNING)\n");
  printf("4: 结束比赛 (GAME_OVER)\n");
  printf("5: 扣血 -100\n");
  printf("6: 耗弹 -20\n");
  printf("7: 加热 +40\n");
  printf("8: 重置状态\n");
  printf("q: 退出\n");
  printf("============================\n");

  while (rclcpp::ok())
  {
    char key = getch();
    std_msgs::msg::UInt8 msg;
    bool publish = true;

    switch (key)
    {
      case '1': msg.data = 1; break;
      case '2': msg.data = 2; break;
      case '3': msg.data = 3; break;
      case '4': msg.data = 4; break;
      case '5': msg.data = 5; break;
      case '6': msg.data = 6; break;
      case '7': msg.data = 7; break;
      case '8': msg.data = 8; break;
      case 'q':
        rclcpp::shutdown();
        printf("退出\n");
        return 0;
      default:
        publish = false;
        break;
    }

    if (publish)
    {
      pub->publish(msg);
      printf("发送指令: %d\n", msg.data);
    }
  }

  return 0;
}
