cmake_minimum_required(VERSION 3.5)

project(ros_bridge VERSION 10.0.0)

# find_package(PROJ REQUIRED)
include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/proj.cmake)

include_directories(
  ${PROJ_INCLUDE_DIRS}
  ${GperfTools_INCLUDE_DIRS}
)

link_directories(
  ${PROJ_LIBRARY_DIRS}
  ${GperfTools_LIBRARY_DIRS}
)

# add_subdirectory(proto)

file(GLOB_RECURSE ROS_BRIDGE_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)

list(FILTER ROS_BRIDGE_SOURCES EXCLUDE REGEX ".*\\.pb\\.cc$")

# foreach(file IN LISTS ROS_BRIDGE_SOURCES)
#   message(STATUS "Cyber Ros Bridge: ${file}")
# endforeach()

add_executable(ros_bridge
  ${ROS_BRIDGE_SOURCES}
)

target_link_libraries(ros_bridge
  ${DEPENCENCY_LIBS} 
  cyber 
  # cyber_proto
  # cyber_ros_bridge_proto
  ${PROJ_LIBRARIES}
  ${GperfTools_LIBRARIES}
  pthread
  fastrtps fastcdr
)

set_target_properties(ros_bridge PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/cyber/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/cyber/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/cyber/${PROJECT_NAME}
  RUNTIME DESTINATION bin/cyber/${PROJECT_NAME}
)