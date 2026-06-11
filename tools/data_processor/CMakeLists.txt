cmake_minimum_required(VERSION 3.16)

project(data_processor)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)

# 包含源代码目录
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(${CMAKE_SOURCE_DIR})
# include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
set(APOLLO_SOURCES
    # ${CMAKE_SOURCE_DIR}/cyber/binary.cc
    # ${CMAKE_SOURCE_DIR}/modules/perception/common/base/camera.cc
    # ${CMAKE_SOURCE_DIR}/modules/perception/common/base/distortion_model.cc
    # ${CMAKE_SOURCE_DIR}/modules/perception/common/base/fisheye_model.cc
)
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)

set(CODEX_SOURCES
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/common/undistortion_handler.cc
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/params/utils.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/params/camera_params.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/tools/opencv/colors.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_2d.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_3d.cpp
)

file(GLOB SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/data_processor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/config/sensor_config.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/config/runtime_config.cpp
)

option(BUILD_ROS1 "Enable ROS1" OFF)

if(BUILD_ROS1)
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
    pcl_conversions
  )

  add_executable(data_processor_ros1
    run_data_processor.cpp
    ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_convert.cpp
    ${CMAKE_SOURCE_DIR}/tools/common/utils/rosbag_utils.cpp
  )

  ## 包含头文件目录
  target_include_directories(data_processor_ros1 PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_message
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_message/version_1.0
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_message/version_1.1
    ${catkin_INCLUDE_DIRS}
  )

  ## 链接 ROS 库
  target_link_libraries(data_processor_ros1
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    cyber_binary
    cyber_common
    modules_common
    perception_common
    ${catkin_LIBRARIES}
  )

  install_module(data_processor_ros1 tools/data_processor
    ros1_message
    ros2_message
    rs_driver
  )
endif()

option(BUILD_ROS2 "Enable ROS1" OFF)

if(BUILD_ROS2)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # 通过 ament_cmake 添加 ROS 支持
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosbag2_cpp REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(pcl_conversions REQUIRED)

  add_executable(data_processor_ros2
    run_data_processor.cpp
    ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_convert.cpp
    ${CMAKE_SOURCE_DIR}/tools/common/utils/rosbag_utils.cpp
    ${CMAKE_SOURCE_DIR}/tools/common/utils/rclcpp_utils.cpp
  )

  target_include_directories(data_processor_ros2 PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_message
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_message/version_1.1
  )

  ## 链接 非ROS 库
  target_link_libraries(data_processor_ros2
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    cyber_binary
    cyber_common
    modules_common
    perception_common
  )

  ## 链接 ROS 库
  ament_target_dependencies(data_processor_ros2
  PUBLIC
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    rosbag2_cpp
    cv_bridge
    image_transport
    pcl_conversions
  )

  ## 链接 ROS MSG 库
  # find_package(ars548_interface REQUIRED)
  # find_package(rslidar_msg REQUIRED)
  # ament_target_dependencies(data_processor_ros2 ars548_interface rslidar_msg)

  find_package(rosidl_typesupport_cpp REQUIRED)
  find_package(rosidl_runtime_cpp REQUIRED)

  target_link_directories(data_processor_ros2 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_message/ars548_interface/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_message/rslidar_msg/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_message/version_1.1/self_state/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_message/version_1.1/sensor/lib
  )

  target_link_libraries(data_processor_ros2 PUBLIC
    ars548_interface__rosidl_typesupport_cpp
    ars548_interface__rosidl_typesupport_c
    ars548_interface__rosidl_typesupport_fastrtps_cpp
    ars548_interface__rosidl_typesupport_introspection_cpp
    rslidar_msg__rosidl_typesupport_cpp
    rslidar_msg__rosidl_typesupport_c
    rslidar_msg__rosidl_typesupport_fastrtps_cpp
    rslidar_msg__rosidl_typesupport_introspection_cpp
    self_state__rosidl_typesupport_cpp
    self_state__rosidl_typesupport_c
    self_state__rosidl_typesupport_fastrtps_cpp
    self_state__rosidl_typesupport_introspection_cpp
    sensor__rosidl_typesupport_cpp
    sensor__rosidl_typesupport_c
    sensor__rosidl_typesupport_fastrtps_cpp
    sensor__rosidl_typesupport_introspection_cpp

    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
  )

  install_module(data_processor_ros2 tools/data_processor
    ros1_message
    ros2_message
    rs_driver
  )

  ament_package()
endif()

# # 包含头文件目录
# target_include_directories(${PROJECT_NAME} PRIVATE 
#   ${CMAKE_SOURCE_DIR}
# )

# # 包含库文件目录
# target_link_libraries(${PROJECT_NAME} 
#   ${PCL_LIBRARIES}
#   pcl_common
#   pcl_io
#   pcl_filters
#   ${OpenCV_LIBRARIES}
#   glog
#   ${Boost_LIBRARIES} boost_thread
#   yaml-cpp
# )