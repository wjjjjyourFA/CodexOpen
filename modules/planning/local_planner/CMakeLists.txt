cmake_minimum_required(VERSION 3.16)

project(local_planner)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT "${CODEXOPEN_ROOT}/ros1/src/planning/local_planner")

find_package(PCL REQUIRED)

add_library(local_planner_core STATIC
  local_path_v2.cpp
)
target_compile_features(local_planner_core PUBLIC cxx_std_14)
target_compile_options(local_planner_core PRIVATE -O3 -Wall -Wextra)
target_include_directories(local_planner_core PUBLIC
  ${CODEXOPEN_ROOT}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(local_planner_core PUBLIC
  ${PCL_LIBRARIES}
)
install_module(local_planner_core modules/planning/local_planner)

if(BUILD_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    std_msgs
  )

  add_executable(local_planner_ros1
    ${ROS1_ADAPTER_ROOT}/src/ros1_convert.cpp
    ${ROS1_ADAPTER_ROOT}/src/run_local_planner_ros1.cpp
  )
  target_compile_features(local_planner_ros1 PRIVATE cxx_std_14)
  target_compile_options(local_planner_ros1 PRIVATE -O3 -Wall -Wextra)
  target_include_directories(local_planner_ros1 PRIVATE
    ${CODEXOPEN_ROOT}
    ${ROS1_ADAPTER_ROOT}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(local_planner_ros1 PRIVATE
    codexopen_ros1_yaml_param_loader
    local_planner_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  install_module(local_planner_ros1 modules/planning/local_planner)
  install(DIRECTORY config/runtime/
    DESTINATION bin/config/LocalPlanner
  )
endif()
