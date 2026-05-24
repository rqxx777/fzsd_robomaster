cmake_minimum_required(VERSION 3.8)
project(armor_detect_ekf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "Root directory of ONNX Runtime")
if(NOT ONNXRUNTIME_ROOT)
  foreach(candidate
    "/home/lby/onnxruntime-linux-x64-1.20.0"
    "/home/lby/onnxruntime-linux-x64-1.20.1"
    "/home/lby/onnxruntime"
  )
    if(EXISTS "${candidate}")
      set(ONNXRUNTIME_ROOT "${candidate}")
      break()
    endif()
  endforeach()
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS
    "${ONNXRUNTIME_ROOT}/include"
    "${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session"
)

find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS
    "${ONNXRUNTIME_ROOT}/lib"
    "${ONNXRUNTIME_ROOT}/build/Linux/Release"
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime not found. Set ONNXRUNTIME_ROOT or install a compatible runtime.")
endif()

get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
message(STATUS "Using ONNX Runtime include dir: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "Using ONNX Runtime library: ${ONNXRUNTIME_LIBRARY}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera_node src/camera_node.cpp)
add_executable(detect_ekf_node src/detect_ekf_node.cpp src/EKF.cpp src/armor_core.cpp)
add_executable(ballistic_node src/ballistic_node.cpp)

ament_target_dependencies(camera_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  visualization_msgs
)

ament_target_dependencies(detect_ekf_node
  ament_index_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  visualization_msgs
)

ament_target_dependencies(ballistic_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(camera_node
  ${OpenCV_LIBS}
)

target_link_libraries(detect_ekf_node
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARY}
  ${cpp_typesupport_target}
)

target_link_libraries(ballistic_node
  ${cpp_typesupport_target}
)

target_include_directories(camera_node PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

target_include_directories(detect_ekf_node PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

target_include_directories(ballistic_node PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

set_target_properties(camera_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
)

set_target_properties(detect_ekf_node PROPERTIES
  INSTALL_RPATH "$ORIGIN;${ONNXRUNTIME_LIBRARY_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS
  camera_node
  detect_ekf_node
  ballistic_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/best.onnx
  DESTINATION share/${PROJECT_NAME}/models
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
