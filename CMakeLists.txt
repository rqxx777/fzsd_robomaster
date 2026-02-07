cmake_minimum_required(VERSION 3.8)
project(armor_detect_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
set(ONNXRUNTIME_DIR "/home/lby/onnxruntime-linux-x64-gpu-1.20.0")
include_directories(${ONNXRUNTIME_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(camera_node src/camera_node.cpp )
add_executable(detect_ekf_node src/detect_ekf_node.cpp src/EKF.cpp src/armor_core.cpp )
ament_target_dependencies(camera_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  visualization_msgs
)

target_link_libraries(camera_node
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime_providers_shared.so
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime_providers_cuda.so
)
ament_target_dependencies(detect_ekf_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  visualization_msgs
)
target_link_libraries(detect_ekf_node
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime_providers_shared.so
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime_providers_cuda.so
)
set_target_properties(camera_node PROPERTIES
    INSTALL_RPATH "$ORIGIN;${ONNXRUNTIME_DIR}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)
set_target_properties(detect_ekf_node PROPERTIES
    INSTALL_RPATH "$ORIGIN;${ONNXRUNTIME_DIR}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)
install(TARGETS
  camera_node
  detect_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()