# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(perception_common_camera_parameter)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cyber)

