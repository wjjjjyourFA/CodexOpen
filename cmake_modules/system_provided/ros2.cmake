### ROS2 ###
option(BUILD_ROS2 "Enable ROS1" OFF)

if(BUILD_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)

  # include_directories(${rclcpp_INCLUDE_DIRS})
endif()