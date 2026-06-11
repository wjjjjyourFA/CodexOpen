# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(perception_common_fusion)

add_subdirectory(lidar2camera)
add_subdirectory(radar2camera)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    perception_common_fusion_lidar2camera
    perception_common_fusion_radar2camera
)