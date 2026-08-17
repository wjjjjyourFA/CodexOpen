cmake_minimum_required(VERSION 3.16)

project(fast_lio_core)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH
  "${CODEXOPEN_ROOT}/cmake_modules/system_provided"
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io visualization)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)

add_library(${PROJECT_NAME} STATIC
  fast_lio_pipeline.cpp
  lidar_odometry.cpp
  src/imu_processing.cpp
  config/runtime_config.cpp
  config/static_config.cpp
  ${CODEXOPEN_ROOT}/modules/common/config/config_file_yaml.cpp
  ${CODEXOPEN_ROOT}/modules/perception/common/algorithm/point_cloud_processing/ikd-Tree/ikd_Tree.cpp
  ${CODEXOPEN_ROOT}/cyber/common/file.cc
  ${CODEXOPEN_ROOT}/cyber/binary.cc
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -w)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)
include(ProcessorCount)
ProcessorCount(FAST_LIO_PROCESSOR_COUNT)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  if(FAST_LIO_PROCESSOR_COUNT GREATER 4)
    target_compile_definitions(${PROJECT_NAME} PRIVATE MP_EN MP_PROC_NUM=3)
  elseif(FAST_LIO_PROCESSOR_COUNT GREATER 3)
    target_compile_definitions(${PROJECT_NAME} PRIVATE MP_EN MP_PROC_NUM=2)
  else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE MP_PROC_NUM=1)
  endif()
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE MP_PROC_NUM=1)
endif()
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CODEXOPEN_ROOT}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/IKFoM_toolkit>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PCL_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
    ${GLOG_LIBRARIES}
    OpenMP::OpenMP_CXX
)

if(COMMAND install_module)
  install_module(${PROJECT_NAME} modules/localization/fast_lio)
endif()

if(BUILD_ROS1)
  set(FAST_LIO_ROS1_ROOT
    "${CODEXOPEN_ROOT}/ros1/src/localization/fast_lio")

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

  add_executable(robot_dog_fast_lio_ros1
    ${FAST_LIO_ROS1_ROOT}/src/ros1_convert.cpp
    ${FAST_LIO_ROS1_ROOT}/src/run_robot_dog_fast_lio.cpp
  )
  target_compile_features(robot_dog_fast_lio_ros1 PRIVATE cxx_std_17)
  target_compile_options(robot_dog_fast_lio_ros1 PRIVATE -O3 -Wall -Wextra)
  target_include_directories(robot_dog_fast_lio_ros1 PRIVATE
    ${FAST_LIO_ROS1_ROOT}/include
    ${ROS1_COMMON_ROOT}/include
    ${CODEXOPEN_ROOT}
    ${catkin_INCLUDE_DIRS}
    ${LIVOX_CUSTOM_MSG_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(robot_dog_fast_lio_ros1 PRIVATE
    fast_lio_core
    codexopen_ros1_yaml_param_loader
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
  )
  install_module(robot_dog_fast_lio_ros1 modules/localization/fast_lio)
  install(DIRECTORY config/runtime/
    DESTINATION bin/config/RobotDogFastLio
  )
endif()
