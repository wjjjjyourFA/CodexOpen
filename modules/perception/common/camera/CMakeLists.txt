cmake_minimum_required(VERSION 3.16)

project(perception_common_camera)

add_subdirectory(params)
add_subdirectory(common)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    perception_common_camera_params
    perception_common_camera_common
)