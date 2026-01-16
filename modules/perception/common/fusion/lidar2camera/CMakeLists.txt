# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(perception_common_fusion_lidar2camera)

set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})

cmake_policy(SET CMP0074 NEW)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/modules/perception/common/base/camera.cc
  ${CMAKE_SOURCE_DIR}/modules/perception/common/base/distortion_model.cc
  ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/common/undistortion_handler_legacy.cc
  ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/parameter/read_camera_params.cpp
  ${CMAKE_SOURCE_DIR}/modules/perception/common/base/parameter/utils.cpp
  ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/colors.cpp
  ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_2d.cpp
  ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_3d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lidar_camera_fusion_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lidar_camera_fusion.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/params/params.cpp
)
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)

add_executable(lidar_camera_fusion_test
  ${SOURCES}
)

target_link_libraries(lidar_camera_fusion_test PRIVATE 
  ${PCL_LIBRARIES}
  # pcl_common
  # pcl_io
  # pcl_filters
  ${OpenCV_LIBRARIES}
  glog
  ${Boost_LIBRARIES} boost_thread
)

if (OPENMP_FOUND)
  target_compile_options(lidar_camera_fusion_test PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(lidar_camera_fusion_test PRIVATE gomp)
endif()