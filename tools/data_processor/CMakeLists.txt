cmake_minimum_required(VERSION 3.5)

project(data_processor_ros1)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})

cmake_policy(SET CMP0074 NEW)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)

# 通过 catkin 添加 ROS 支持
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  rosbag
  cv_bridge
  image_transport
)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ros1_message)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
set(SOURCES
    ${CMAKE_SOURCE_DIR}/cyber/binary.cc
    ${CMAKE_SOURCE_DIR}/modules/perception/common/base/camera.cc
    ${CMAKE_SOURCE_DIR}/modules/perception/common/base/distortion_model.cc
    ${CMAKE_SOURCE_DIR}/modules/perception/common/base/parameter/utils.cpp
    ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/common/undistortion_handler_legacy.cc
    ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/parameter/read_camera_params.cpp
    ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/colors.cpp
    ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_2d.cpp
    ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/data_processor_ros1.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/params/params.cpp
)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)

add_executable(${PROJECT_NAME}
    ${SOURCES}
)

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/message
)

# 包含库文件目录
target_link_libraries(${PROJECT_NAME} 
    ${PCL_LIBRARIES}
    pcl_common
    pcl_io
    pcl_filters
    ${OpenCV_LIBRARIES}
    glog
    ${Boost_LIBRARIES} boost_thread
    yaml-cpp
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 链接ROS库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)