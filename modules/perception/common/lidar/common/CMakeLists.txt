cmake_minimum_required(VERSION 3.16)

project(perception_common_lidar_common)

add_library(${PROJECT_NAME} STATIC
  motion_ompensator.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  common_math
  perception_common_config
)

install_module(${PROJECT_NAME} modules/perception/common/lidar/common)
