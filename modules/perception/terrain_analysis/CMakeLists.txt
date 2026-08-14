cmake_minimum_required(VERSION 3.16)

project(terrain_analysis)

if(NOT BUILD_ROS1)
  message(STATUS "terrain_analysis realtime program requires BUILD_ROS1=ON")
  return()
endif()

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT "${CODEXOPEN_ROOT}/ros1/src/perception/terrain_analysis")

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(PCL REQUIRED)

add_executable(terrain_analysis_ros1
  terrainAnalysis.cpp
  ${ROS1_ADAPTER_ROOT}/src/run_terrain_analysis_ros1.cpp
)
target_compile_features(terrain_analysis_ros1 PRIVATE cxx_std_14)
target_compile_options(terrain_analysis_ros1 PRIVATE -O3 -Wall)
target_compile_definitions(terrain_analysis_ros1 PRIVATE
  ROBOT_PLAN_EXPV2_NO_MAIN
)
target_include_directories(terrain_analysis_ros1 PRIVATE
  ${CODEXOPEN_ROOT}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(terrain_analysis_ros1 PRIVATE
  codexopen_ros1_yaml_param_loader
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install_module(terrain_analysis_ros1 modules/perception/terrain_analysis)
install(DIRECTORY config/runtime/
  DESTINATION bin/config/TerrainAnalysis
)
