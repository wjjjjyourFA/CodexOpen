cmake_minimum_required(VERSION 3.16)

project(fast_calib_core)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  features
  filters
  io
  registration
  sample_consensus
  search
  segmentation
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_library(${PROJECT_NAME} STATIC
  src/calibration_common.cpp
  src/lidar_detector.cpp
  src/multi_scene_calibration.cpp
  src/qr_detector.cpp
  src/single_calibration.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_compile_definitions(${PROJECT_NAME} PUBLIC PCL_NO_PRECOMPILE)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

if(COMMAND install_module)
  install_module(${PROJECT_NAME} tools/fast_calib src)
endif()
install(FILES LICENSE README.md DESTINATION share/tools/fast_calib)

if(BUILD_ROS1)
  set(FAST_CALIB_ROS1_ROOT
    "${CODEXOPEN_ROOT}/ros1/src/tools/fast_calib")
  find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    roscpp
    rosbag
    sensor_msgs
    std_msgs
  )
  find_package(yaml-cpp REQUIRED)

  add_library(fast_calib_ros1_adapter STATIC
    ${FAST_CALIB_ROS1_ROOT}/src/bag_loader.cpp
    ${FAST_CALIB_ROS1_ROOT}/src/config.cpp
  )
  target_compile_features(fast_calib_ros1_adapter PUBLIC cxx_std_14)
  target_compile_definitions(fast_calib_ros1_adapter PUBLIC PCL_NO_PRECOMPILE)
  target_include_directories(fast_calib_ros1_adapter
    PUBLIC
      ${FAST_CALIB_ROS1_ROOT}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CODEXOPEN_ROOT}/tools/data_processor/ros1_message
      ${catkin_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(fast_calib_ros1_adapter
    PUBLIC
      fast_calib_core
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBS}
  )

  add_executable(fast_calib_ros1
    ${FAST_CALIB_ROS1_ROOT}/src/run_fast_calib_ros1.cpp
  )
  target_compile_features(fast_calib_ros1 PRIVATE cxx_std_14)
  target_include_directories(fast_calib_ros1 PRIVATE
    ${FAST_CALIB_ROS1_ROOT}/include
    ${ROS1_COMMON_ROOT}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(fast_calib_ros1 PRIVATE
    fast_calib_ros1_adapter
    codexopen_ros1_yaml_param_loader
    ${catkin_LIBRARIES}
    yaml-cpp
  )

  add_executable(multi_fast_calib_ros1
    ${FAST_CALIB_ROS1_ROOT}/src/run_multi_fast_calib_ros1.cpp
  )
  target_compile_features(multi_fast_calib_ros1 PRIVATE cxx_std_14)
  target_include_directories(multi_fast_calib_ros1 PRIVATE
    ${FAST_CALIB_ROS1_ROOT}/include
    ${ROS1_COMMON_ROOT}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(multi_fast_calib_ros1 PRIVATE
    fast_calib_ros1_adapter
    codexopen_ros1_yaml_param_loader
    ${catkin_LIBRARIES}
    yaml-cpp
  )

  install_module(fast_calib_ros1 tools/fast_calib src)
  install_module(multi_fast_calib_ros1 tools/fast_calib src)
  install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/distance_filter_tool.py
    DESTINATION bin/tools/fast_calib
  )

  # Per project convention these are also the source-of-truth files. There is
  # deliberately no config directory inside tools/fast_calib.
  set(FAST_CALIB_CONFIG_ROOT
    "${CODEXOPEN_ROOT}/install/bin/config/FastCalib")
  foreach(CONFIG_FILE FastCalib.yaml Interface.yaml rviz/FastCalib.rviz)
    if(NOT EXISTS "${FAST_CALIB_CONFIG_ROOT}/${CONFIG_FILE}")
      message(FATAL_ERROR
        "Missing required FAST-Calib config: ${FAST_CALIB_CONFIG_ROOT}/${CONFIG_FILE}")
    endif()
  endforeach()
  install(FILES
    ${FAST_CALIB_CONFIG_ROOT}/FastCalib.yaml
    ${FAST_CALIB_CONFIG_ROOT}/Interface.yaml
    DESTINATION bin/config/FastCalib
  )
  install(FILES
    ${FAST_CALIB_CONFIG_ROOT}/rviz/FastCalib.rviz
    DESTINATION bin/config/FastCalib/rviz
  )
endif()
