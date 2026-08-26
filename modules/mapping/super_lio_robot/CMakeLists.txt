cmake_minimum_required(VERSION 3.16)

project(super_lio_robot_core)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH
  "${CODEXOPEN_ROOT}/cmake_modules/system_provided"
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(TBB REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/ESKF.cpp
  src/Manifold.cpp
  src/params.cpp
  src/super_lio.cpp
  src/super_lio_reloc.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -w)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/modules/mapping/super_lio_robot/include>
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    gflags
    TBB::tbb
)

if(COMMAND install_module)
  install_module(${PROJECT_NAME} modules/mapping/super_lio_robot)
endif()
install(FILES LICENSE DESTINATION share/licenses/super_lio_robot)

if(BUILD_ROS1)
  set(SUPER_LIO_ROBOT_ROS1_ROOT
    "${CODEXOPEN_ROOT}/ros1/src/mapping/super_lio_robot")

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    tf
  )
  find_path(LIVOX_CUSTOM_MSG_INCLUDE_DIR
    NAMES
      livox_ros_driver2/CustomMsg.h
      livox_ros_driver/CustomMsg.h
      livox_ros_driver2-1.2.4/CustomMsg.h
    HINTS ${catkin_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}
    PATHS
      /opt/ros/noetic/include
      ${CODEXOPEN_ROOT}/tools/data_processor/ros1_message
    PATH_SUFFIXES include
  )
  if(NOT LIVOX_CUSTOM_MSG_INCLUDE_DIR)
    message(FATAL_ERROR "Livox CustomMsg header was not found")
  endif()

  add_executable(super_lio_robot_ros1
    ${SUPER_LIO_ROBOT_ROS1_ROOT}/src/ros1_convert.cpp
    ${SUPER_LIO_ROBOT_ROS1_ROOT}/src/super_lio_robot_app.cpp
  )
  target_compile_features(super_lio_robot_ros1 PRIVATE cxx_std_17)
  target_compile_options(super_lio_robot_ros1 PRIVATE -O3 -Wall -Wextra)
  target_include_directories(super_lio_robot_ros1 PRIVATE
    ${SUPER_LIO_ROBOT_ROS1_ROOT}/include
    ${ROS1_COMMON_ROOT}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CODEXOPEN_ROOT}
    ${catkin_INCLUDE_DIRS}
    ${LIVOX_CUSTOM_MSG_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(super_lio_robot_ros1 PRIVATE
    super_lio_robot_core
    codexopen_ros1_yaml_param_loader
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
  )
  install_module(super_lio_robot_ros1 modules/mapping/super_lio_robot)
  install(DIRECTORY config/runtime/
    DESTINATION bin/config/SuperLioRobot
  )
endif()
