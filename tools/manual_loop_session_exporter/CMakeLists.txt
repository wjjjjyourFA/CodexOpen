cmake_minimum_required(VERSION 3.16)

project(manual_loop_session_exporter_core)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_library(${PROJECT_NAME} STATIC
  session_exporter.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CODEXOPEN_ROOT}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)

if(COMMAND install_module)
  install_module(${PROJECT_NAME} tools/manual_loop_session_exporter)
endif()

if(BUILD_ROS1)
  set(EXPORTER_ROS1_ROOT
    "${CODEXOPEN_ROOT}/ros1/src/tools/manual_loop_session_exporter")
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_filters
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    std_srvs
  )
  find_package(yaml-cpp REQUIRED)

  add_executable(manual_loop_session_exporter_ros1
    ${EXPORTER_ROS1_ROOT}/src/ros1_convert.cpp
    ${EXPORTER_ROS1_ROOT}/src/run_manual_loop_session_exporter.cpp
  )
  target_compile_features(manual_loop_session_exporter_ros1 PRIVATE cxx_std_14)
  target_compile_options(manual_loop_session_exporter_ros1 PRIVATE -Wall -Wextra)
  target_include_directories(manual_loop_session_exporter_ros1 PRIVATE
    ${EXPORTER_ROS1_ROOT}/include
    ${ROS1_COMMON_ROOT}/include
    ${CODEXOPEN_ROOT}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(manual_loop_session_exporter_ros1 PRIVATE
    manual_loop_session_exporter_core
    codexopen_ros1_yaml_param_loader
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
  )
  install_module(manual_loop_session_exporter_ros1
    tools/manual_loop_session_exporter)
  install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_session.py
    DESTINATION bin/tools/manual_loop_session_exporter
  )
  install(DIRECTORY config/runtime/
    DESTINATION bin/config/ManualLoopSessionExporter
  )
endif()
