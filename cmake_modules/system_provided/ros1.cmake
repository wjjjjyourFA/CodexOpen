### ROS1 ###
option(BUILD_ROS1 "Enable ROS1" OFF)

if(BUILD_ROS1)
  find_package(roscpp 1.12 QUIET)
  find_package(roslib QUIET)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(cv_bridge REQUIRED)

  # include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})

  # catkin use, i don't use it in cmake
  # find_package(catkin REQUIRED COMPONENTS
  #   roscpp
  #   rospy
  #   std_msgs
  #   sensor_msgs
  #   geometry_msgs
  #   nav_msgs
  #   message_generation
  # )

  # ROS include
  set(ROS1_INCLUDE_DIRS
    ${roscpp_INCLUDE_DIRS}
    ${roslib_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/ros1/devel/include
  )

  # ROS library
  set(ROS1_LIBRARIES
    ${roscpp_LIBRARIES}
    ${roslib_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    ${image_transport_LIBRARIES}
    ${cv_bridge_LIBRARIES}
  )
endif()