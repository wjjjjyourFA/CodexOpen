cmake_minimum_required(VERSION 3.16)

project(waypoint_publisher)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT "${CODEXOPEN_ROOT}/ros1/src/planning/waypoint_publisher")

add_library(waypoint_publisher_core STATIC
  waypoint_publisher.cpp
)
target_compile_features(waypoint_publisher_core PUBLIC cxx_std_14)
target_compile_options(waypoint_publisher_core PRIVATE -O3 -Wall -Wextra)
target_include_directories(waypoint_publisher_core PUBLIC
  ${CODEXOPEN_ROOT}
)
install_module(waypoint_publisher_core modules/planning/waypoint_publisher)

if(BUILD_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
  )
  add_executable(waypoint_publisher_ros1
    ${ROS1_ADAPTER_ROOT}/src/ros1_convert.cpp
    ${ROS1_ADAPTER_ROOT}/src/run_waypoint_publisher_ros1.cpp
  )
  target_compile_features(waypoint_publisher_ros1 PRIVATE cxx_std_14)
  target_compile_options(waypoint_publisher_ros1 PRIVATE -O3 -Wall -Wextra)
  target_include_directories(waypoint_publisher_ros1 PRIVATE
    ${CODEXOPEN_ROOT}
    ${ROS1_ADAPTER_ROOT}/include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(waypoint_publisher_ros1 PRIVATE
    codexopen_ros1_yaml_param_loader
    waypoint_publisher_core
    ${catkin_LIBRARIES}
  )
  install_module(waypoint_publisher_ros1 modules/planning/waypoint_publisher)
  install(DIRECTORY config/runtime/
    DESTINATION bin/config/WaypointPublisher
  )
endif()
