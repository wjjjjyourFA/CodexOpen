cmake_minimum_required(VERSION 3.16)

project(terrain_waypoint_exploration)

if(NOT BUILD_ROS1)
  message(STATUS "terrain_waypoint_exploration realtime program requires BUILD_ROS1=ON")
  return()
endif()

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT
  "${CODEXOPEN_ROOT}/ros1/src/planning/terrain_waypoint_exploration")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)
find_package(PCL REQUIRED)

add_executable(terrain_waypoint_exploration_ros1
  src/terrain_waypoint_explorer.cpp
  ${ROS1_ADAPTER_ROOT}/src/run_terrain_waypoint_exploration_ros1.cpp
)
target_compile_features(terrain_waypoint_exploration_ros1 PRIVATE cxx_std_14)
target_compile_options(terrain_waypoint_exploration_ros1 PRIVATE -O3 -Wall)
target_compile_definitions(terrain_waypoint_exploration_ros1 PRIVATE
  ROBOT_PLAN_EXPV2_NO_MAIN
)
target_include_directories(terrain_waypoint_exploration_ros1 PRIVATE
  ${CODEXOPEN_ROOT}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(terrain_waypoint_exploration_ros1 PRIVATE
  codexopen_ros1_yaml_param_loader
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install_module(terrain_waypoint_exploration_ros1
  modules/planning/terrain_waypoint_exploration)
install(DIRECTORY config/runtime/
  DESTINATION bin/config/TerrainWaypointExploration
)
