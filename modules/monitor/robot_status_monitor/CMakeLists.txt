cmake_minimum_required(VERSION 3.16)

project(robot_status_monitor)

if(BUILD_ROS1)
  get_filename_component(CODEXOPEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  install(PROGRAMS
    ${CODEXOPEN_ROOT}/ros1/src/monitor/robot_status_monitor/scripts/robot_status_monitor_node.py
    DESTINATION bin/modules/monitor/robot_status_monitor
    RENAME robot_status_monitor_ros1
  )
  install(DIRECTORY config/
    DESTINATION bin/config/RobotStatusMonitor
  )
endif()
