cmake_minimum_required(VERSION 3.16)

project(codexopen_ros1_common)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(yaml-cpp REQUIRED)

add_library(codexopen_ros1_yaml_param_loader STATIC
  src/yaml_param_loader.cpp
)
target_compile_features(codexopen_ros1_yaml_param_loader PUBLIC cxx_std_14)
target_include_directories(codexopen_ros1_yaml_param_loader
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(codexopen_ros1_yaml_param_loader
  PUBLIC
    ${catkin_LIBRARIES}
    yaml-cpp
)
