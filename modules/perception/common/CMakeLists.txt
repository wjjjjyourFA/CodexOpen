cmake_minimum_required(VERSION 3.16)

project(perception_common)

add_subdirectory(base)
add_subdirectory(config)
add_subdirectory(camera)
add_subdirectory(lidar)
add_subdirectory(fusion)
# add_subdirectory(algorithm)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    perception_common_base
    perception_common_config
    perception_common_camera
    perception_common_lidar
    perception_common_fusion
)