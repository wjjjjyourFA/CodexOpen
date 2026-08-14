cmake_minimum_required(VERSION 3.16)

project(modules)

include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/eigen3.cmake)

add_subdirectory(common)
# add_subdirectory(drivers)
add_subdirectory(localization)
add_subdirectory(monitor)
add_subdirectory(perception)
add_subdirectory(planning)
# add_subdirectory(tools)
