cmake_minimum_required(VERSION 3.16)

project(perception_common_lidar_cluster)

add_library(${PROJECT_NAME} STATIC
  object_cluster.cpp
  ${CMAKE_SOURCE_DIR}/modules/perception/common/algorithm/point_cloud_processing/kdtree/kdtree.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  common_math
  perception_common_base
  perception_common_algorithm_point_cloud_processing
)

install_module(${PROJECT_NAME} modules/perception/common/lidar/cluster)
