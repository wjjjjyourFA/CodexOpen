cmake_minimum_required(VERSION 3.16)

project(waypoint_publisher)

if(NOT BUILD_ROS1)
  message(STATUS "waypoint_publisher realtime program requires BUILD_ROS1=ON")
  return()
endif()

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT "${CODEXOPEN_ROOT}/ros1/src/planning/waypoint_publisher")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
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

add_executable(waypoint_publisher_ros1
  waypointExample.cpp
  ${ROS1_ADAPTER_ROOT}/src/run_waypoint_publisher_ros1.cpp
)
target_compile_features(waypoint_publisher_ros1 PRIVATE cxx_std_14)
target_compile_options(waypoint_publisher_ros1 PRIVATE -O3 -Wall)
target_compile_definitions(waypoint_publisher_ros1 PRIVATE
  WAYPOINT_PUBLISHER_NO_MAIN
)
target_include_directories(waypoint_publisher_ros1 PRIVATE
  ${CODEXOPEN_ROOT}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(waypoint_publisher_ros1 PRIVATE
  codexopen_ros1_yaml_param_loader
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install_module(waypoint_publisher_ros1 modules/planning/waypoint_publisher)
install(DIRECTORY config/runtime/
  DESTINATION bin/config/WaypointPublisher
)
