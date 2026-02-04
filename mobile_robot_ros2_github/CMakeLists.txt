cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

# 默认 C++17
set(CMAKE_CXX_STANDARD 17)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# 键盘控制节点
add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop
  rclcpp
  geometry_msgs
)

# 车轮模拟器节点
add_executable(wheel_simulator src/wheel_simulator.cpp)
ament_target_dependencies(wheel_simulator
  rclcpp
  geometry_msgs
  sensor_msgs
)

# 安装
install(TARGETS teleop wheel_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装其他文件
install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()