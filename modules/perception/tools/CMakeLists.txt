cmake_minimum_required(VERSION 3.16)

project(perception_tools)

add_subdirectory(opencv)
add_subdirectory(pcl)
add_subdirectory(common)

add_library(perception_tools INTERFACE)

target_link_libraries(perception_tools
  INTERFACE
    perception_tools_opencv
    perception_tools_pcl
    perception_tools_common
)
