cmake_minimum_required(VERSION 3.16)

project(prior_map_localization)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT
  "${CODEXOPEN_ROOT}/ros1/src/localization/prior_map_localization")
list(APPEND CMAKE_MODULE_PATH
  "${CODEXOPEN_ROOT}/cmake_modules/system_provided"
)

find_package(Boost REQUIRED COMPONENTS chrono serialization thread timer)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common filters io kdtree registration visualization)
find_package(yaml-cpp REQUIRED)
add_library(prior_map_localization_core STATIC
  prior_map_localization.cpp
  src/ImuProcess.cpp
  src/MapProcess.cpp
  src/PreProcess.cpp
  include/ikd-Tree/ikd_Tree.cpp
)
target_compile_features(prior_map_localization_core PUBLIC cxx_std_17)
target_compile_options(prior_map_localization_core PRIVATE -O3 -w)
target_compile_definitions(prior_map_localization_core PUBLIC
  ROOT_DIR="${CMAKE_INSTALL_PREFIX}/bin/config/PriorMapLocalization/"
)

include(ProcessorCount)
ProcessorCount(PRIOR_MAP_LOCALIZATION_PROCESSOR_COUNT)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  if(PRIOR_MAP_LOCALIZATION_PROCESSOR_COUNT GREATER 5)
    target_compile_definitions(prior_map_localization_core PUBLIC
      MP_EN MP_PROC_NUM=4)
  elseif(PRIOR_MAP_LOCALIZATION_PROCESSOR_COUNT GREATER 3)
    math(EXPR PRIOR_MAP_LOCALIZATION_MP_PROC_NUM
      "${PRIOR_MAP_LOCALIZATION_PROCESSOR_COUNT} - 2")
    target_compile_definitions(prior_map_localization_core PUBLIC
      MP_EN MP_PROC_NUM=${PRIOR_MAP_LOCALIZATION_MP_PROC_NUM})
  else()
    target_compile_definitions(prior_map_localization_core PUBLIC MP_PROC_NUM=1)
  endif()
else()
  target_compile_definitions(prior_map_localization_core PUBLIC MP_PROC_NUM=1)
endif()

target_include_directories(prior_map_localization_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/IKFoM_toolkit
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(prior_map_localization_core
  PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GLOG_LIBRARIES}
    yaml-cpp
    OpenMP::OpenMP_CXX
)

install_module(prior_map_localization_core
  modules/localization/prior_map_localization)
install(DIRECTORY config/runtime/
  DESTINATION bin/config/PriorMapLocalization
)

if(BUILD_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_geometry_msgs
  )

  add_executable(localization_robot_ros1
    ${ROS1_ADAPTER_ROOT}/src/ros1_convert.cpp
    ${ROS1_ADAPTER_ROOT}/src/run_prior_map_localization_ros1.cpp
  )
  target_compile_features(localization_robot_ros1 PRIVATE cxx_std_17)
  target_compile_options(localization_robot_ros1 PRIVATE -O3 -w)
  target_include_directories(localization_robot_ros1 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/IKFoM_toolkit
    ${ROS1_ADAPTER_ROOT}/include
    ${CODEXOPEN_ROOT}/ros1/src/common/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(localization_robot_ros1 PRIVATE
    prior_map_localization_core
    codexopen_ros1_yaml_param_loader
    ${catkin_LIBRARIES}
  )

  install_module(localization_robot_ros1
    modules/localization/prior_map_localization)
  install(DIRECTORY
    ${ROS1_ADAPTER_ROOT}/include/self_state
    ${ROS1_ADAPTER_ROOT}/include/sensor
    DESTINATION include/modules/localization/prior_map_localization/include
  )
endif()
