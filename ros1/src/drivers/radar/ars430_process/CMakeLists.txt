cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)

project(ars430_process)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  pcl_conversions
  pcl_ros
  tf
  visualization_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RadarPacket.msg
  SensorStatus.msg
  RadarDetection.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime
  # DEPENDS system_lib
)

# JOJO
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/CodexOpen)

set(JOJO_SOURCES
  ${PROJECT_SOURCE_DIR}/config/config_manager.cpp
  ${PROJECT_SOURCE_DIR}/config/runtime_config.cpp
  ${PROJECT_SOURCE_DIR}/config/vehicle_model_config.cpp
  ${SERIAL_LIB_SRCS}
)

## Vars to build with
set(extra_LIBRARIES -lpcap -lpthread) #Packet Cap, Capabilities, Mutexes
set(CMAKE_CXX_FLAGS "-std=c++14 -w -g -O3 ${CMAKE_CXX_FLAGS}") #Ignore all warnings, debug flags, optimize
# set(RADAR_UNFILTER_SOURCE 
#   src/UnfilteredMain.cpp 
#   src/parser.cpp 
#   src/sniffer.cpp
# )
set(RADAR_UNFILTER_SOURCE 
  src/ars430_process_node.cpp    
  src/data_process.cpp
  src/sniffer.cpp
  ${JOJO_SOURCES}
)
set(RADAR_FILTER_SOURCE
  src/info_convert_node.cpp
  src/processPacket.cpp
)
set(VIS_SOURCE
  src/PointCloudConverter.cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include  ${catkin_INCLUDE_DIRS})

## Unfiltered Publisher
add_executable(${PROJECT_NAME}_node ${RADAR_UNFILTER_SOURCE} ${JOJO_SOURCES})

target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
                      ${extra_LIBRARIES}
                      yaml-cpp
                      )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

## Filtered Publisher
add_executable(info_convert_node ${RADAR_FILTER_SOURCE} ${JOJO_SOURCES})

target_link_libraries(info_convert_node
                      ${catkin_LIBRARIES}
                      ${extra_LIBRARIES}
                      yaml-cpp
                      )

add_dependencies(info_convert_node ${PROJECT_NAME}_generate_messages_cpp)

## Point Cloud Generator
add_executable(radar_visualizer ${VIS_SOURCE} ${JOJO_SOURCES})

target_link_libraries(radar_visualizer ${catkin_LIBRARIES} yaml-cpp)

add_dependencies(radar_visualizer ${PROJECT_NAME}_generate_messages_cpp)
