cmake_minimum_required(VERSION 3.8)
project(rm_referee_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameStatus.msg"
  "msg/RobotStatus.msg"
  "msg/Decision.msg"
)

ament_package()
