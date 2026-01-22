cmake_minimum_required(VERSION 3.10)
project(opencv_test)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

file(GLOB SOURCES "src/*.cpp")
add_executable(main ${SOURCES})

set(ONNXRUNTIME_DIR "/home/lby/onnxruntime-linux-x64-gpu-1.20.0")
include_directories(${ONNXRUNTIME_DIR}/include)

target_link_libraries(main
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime_providers_shared.so
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime_providers_cuda.so
)
