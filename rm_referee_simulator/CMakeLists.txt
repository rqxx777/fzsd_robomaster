cmake_minimum_required(VERSION 3.8)
project(rm_referee_simulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_referee_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(referee_simulator_node
  src/referee_simulator_node.cpp
)
ament_target_dependencies(referee_simulator_node
  rclcpp
  rm_referee_interfaces
  std_msgs
)

add_executable(referee_keyboard
  src/referee_keyboard.cpp
)
ament_target_dependencies(referee_keyboard
  rclcpp
  std_msgs
)

add_executable(decision_node
  src/decision_node.cpp
)
ament_target_dependencies(decision_node
  rclcpp
  rm_referee_interfaces
)

add_executable(referee_executor
  src/referee_executor.cpp
)
ament_target_dependencies(referee_executor
  rclcpp
  rclcpp_action
  rm_referee_interfaces
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  referee_simulator_node
  referee_keyboard
  decision_node
  referee_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
