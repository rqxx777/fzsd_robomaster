cmake_minimum_required(VERSION 3.8)
project(vision_attacker)

set(CMAKE_CXX_STANDARD 17)

# Option to disable lint/format tests (uncrustify/ament_lint_auto)
option(DISABLE_LINT "Disable ament_lint_auto checks (uncrustify, lint_cmake, etc.)" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(src/tinympc)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(SRCS
  src/planner.cpp
  src/target.cpp
  src/math_tools.cpp
  src/trajectory.cpp
)

# Build a library for the core vision_attacker logic so tests can link against it
add_library(vision_attacker_lib STATIC ${SRCS})
target_include_directories(vision_attacker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vision_attacker_lib tinympcstatic Eigen3::Eigen ${OpenCV_LIBS})
ament_target_dependencies(vision_attacker_lib
  rclcpp
  vision_interfaces
  auto_aim_interfaces
  tf2
  tf2_ros
  angles
  visualization_msgs)

add_executable(vision_attacker_node src/vision_attacker_node.cpp)
target_include_directories(vision_attacker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vision_attacker_node vision_attacker_lib)

ament_target_dependencies(vision_attacker_node
  rclcpp
  vision_interfaces
  auto_aim_interfaces
  tf2
  tf2_ros
  angles
  visualization_msgs)

install(TARGETS vision_attacker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  if(NOT DISABLE_LINT)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner test/test_planner.cpp)
  if(TARGET test_planner)
    target_include_directories(test_planner PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    ament_target_dependencies(test_planner
      auto_aim_interfaces
      rclcpp)
    target_link_libraries(test_planner vision_attacker_lib tinympcstatic Eigen3::Eigen ${OpenCV_LIBS})
  endif()
endif()

ament_package()
