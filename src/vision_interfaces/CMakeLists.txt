cmake_minimum_required(VERSION 3.8)
project(vision_interfaces)
add_compile_options(-Wno-dev)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Robot.msg"
  "msg/AutoAim.msg"
  "msg/GameState.msg"
  DEPENDENCIES 
  geometry_msgs
)

ament_package()
