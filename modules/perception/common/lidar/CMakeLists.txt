cmake_minimum_required(VERSION 3.16)

project(perception_common_lidar)

add_subdirectory(convert)
add_subdirectory(common)
add_subdirectory(cluster)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    perception_common_lidar_convert
    perception_common_lidar_common
    perception_common_lidar_cluster
)
