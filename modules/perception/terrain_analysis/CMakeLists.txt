cmake_minimum_required(VERSION 3.16)

project(terrain_analysis)

get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROS1_ADAPTER_ROOT "${CODEXOPEN_ROOT}/ros1/src/perception/terrain_analysis")

find_package(PCL REQUIRED)

add_library(terrain_analysis_core STATIC
  terrain_analysis.cpp
)
target_compile_features(terrain_analysis_core PUBLIC cxx_std_14)
target_compile_options(terrain_analysis_core PRIVATE -O3 -Wall -Wextra)
target_include_directories(terrain_analysis_core PUBLIC
  ${CODEXOPEN_ROOT}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(terrain_analysis_core PUBLIC
  ${PCL_LIBRARIES}
)
install_module(terrain_analysis_core modules/perception/terrain_analysis)

if(BUILD_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    std_msgs
  )

  add_executable(terrain_analysis_ros1
    ${ROS1_ADAPTER_ROOT}/src/ros1_convert.cpp
    ${ROS1_ADAPTER_ROOT}/src/run_terrain_analysis_ros1.cpp
  )
  target_compile_features(terrain_analysis_ros1 PRIVATE cxx_std_14)
  target_compile_options(terrain_analysis_ros1 PRIVATE -O3 -Wall -Wextra)
  target_include_directories(terrain_analysis_ros1 PRIVATE
    ${CODEXOPEN_ROOT}
    ${ROS1_ADAPTER_ROOT}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(terrain_analysis_ros1 PRIVATE
    codexopen_ros1_yaml_param_loader
    terrain_analysis_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  install_module(terrain_analysis_ros1 modules/perception/terrain_analysis)
  install(DIRECTORY config/runtime/
    DESTINATION bin/config/TerrainAnalysis
  )
endif()
