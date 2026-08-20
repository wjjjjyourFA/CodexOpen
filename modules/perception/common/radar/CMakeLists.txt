cmake_minimum_required(VERSION 3.16)

project(perception_common_radar)

add_subdirectory(convert)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  perception_common_radar_convert
)
