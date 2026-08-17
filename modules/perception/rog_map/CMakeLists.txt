cmake_minimum_required(VERSION 3.16)

project(rog_map)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT "${CODEXOPEN_ROOT}/ros1/src/perception/rog_map")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_library(DW_LIBRARY NAMES dw)
if(NOT DW_LIBRARY)
  find_file(DW_LIBRARY NAMES libdw.so.1
    PATHS /lib /usr/lib
    PATH_SUFFIXES x86_64-linux-gnu aarch64-linux-gnu
  )
endif()
set(ROG_MAP_OPTIONAL_LIBRARIES)
if(DW_LIBRARY)
  list(APPEND ROG_MAP_OPTIONAL_LIBRARIES ${DW_LIBRARY})
endif()

add_library(rog_map_core STATIC
  include/utils/raycaster.cpp
  src/rog_map/counter_map.cpp
  src/rog_map/esdf_map.cpp
  src/rog_map/inf_map.cpp
  src/rog_map/prob_map.cpp
  src/rog_map/rog_map.cpp
  src/rog_map/sliding_map.cpp
)
target_compile_features(rog_map_core PUBLIC cxx_std_14)
target_compile_options(rog_map_core PRIVATE -O3 -Wall)
target_compile_definitions(rog_map_core PRIVATE
  ORIGIN_AT_CORNER
  QT_NO_VERSION_TAGGING
  ROOT_DIR="${CMAKE_INSTALL_PREFIX}/"
)
target_include_directories(rog_map_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(rog_map_core PUBLIC
  ${PCL_LIBRARIES}
  ${ROG_MAP_OPTIONAL_LIBRARIES}
)
install_module(rog_map_core modules/perception/rog_map)

if(BUILD_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    tf2_ros
    visualization_msgs
  )

  add_executable(rog_map_ros1
    ${ROS1_ADAPTER_ROOT}/src/ros1_convert.cpp
    ${ROS1_ADAPTER_ROOT}/src/run_rog_map_ros1.cpp
  )
  target_compile_features(rog_map_ros1 PRIVATE cxx_std_14)
  target_compile_options(rog_map_ros1 PRIVATE -O3 -Wall -Wextra)
  target_include_directories(rog_map_ros1 PRIVATE
    ${CODEXOPEN_ROOT}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROS1_ADAPTER_ROOT}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(rog_map_ros1 PRIVATE
    codexopen_ros1_yaml_param_loader
    rog_map_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  install_module(rog_map_ros1 modules/perception/rog_map)
  install(DIRECTORY config/runtime/
    DESTINATION bin/config/RogMap
  )
endif()
