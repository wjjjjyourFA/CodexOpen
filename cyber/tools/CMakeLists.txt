# cyber/tools/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_tools VERSION 1.0.0)

find_package(Curses REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GperfTools_INCLUDE_DIRS}
)
link_directories(${GperfTools_LIBRARY_DIRS})

# cyber monitor
add_executable(cyber_monitor
  cyber_monitor/cyber_topology_message.cc
  cyber_monitor/general_channel_message.cc
  cyber_monitor/general_message.cc
  cyber_monitor/general_message_base.cc
  cyber_monitor/renderable_message.cc
  cyber_monitor/screen.cc
  cyber_monitor/main.cc
)
target_link_libraries(cyber_monitor
  ncurses
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
)
set_target_properties(cyber_monitor PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/tools/cyber_monitor/"
)

# cyber_recorder
add_executable(cyber_recorder
  cyber_recorder/main.cc
  cyber_recorder/info.cc
  cyber_recorder/recorder.cc
  cyber_recorder/recoverer.cc
  cyber_recorder/spliter.cc
  cyber_recorder/player/play_task.cc
  cyber_recorder/player/play_task_buffer.cc
  cyber_recorder/player/play_task_consumer.cc
  cyber_recorder/player/play_task_producer.cc
  cyber_recorder/player/player.cc
)
target_link_libraries(cyber_recorder
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
  ${GperfTools_LIBRARIES}
)
set_target_properties(cyber_recorder PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/tools/cyber_recorder/"
)

#### cyber_channel cyber_launch cyber_node cyber_service ####
# 安装 python 工具脚本（移除 .py 后缀）

# file(COPY
#   cyber_channel/cyber_channel.py
#   DESTINATION ${CMAKE_BINARY_DIR}/cyber/tools/cyber_channel/
# )
# file(COPY
#   cyber_launch/cyber_launch.py
#   DESTINATION ${CMAKE_BINARY_DIR}/cyber/tools/cyber_launch/
# )
# file(COPY
#   cyber_node/cyber_node.py
#   DESTINATION ${CMAKE_BINARY_DIR}/cyber/tools/cyber_node/
# )
# file(COPY
#   cyber_service/cyber_service.py
#   DESTINATION ${CMAKE_BINARY_DIR}/cyber/tools/cyber_service/
# )
# file(COPY
#   cyber_tools_auto_complete.bash
#   DESTINATION ${CMAKE_BINARY_DIR}/cyber/tools/
# )

# file(RENAME ${CMAKE_BINARY_DIR}/cyber/tools/cyber_channel/cyber_channel.py ${CMAKE_BINARY_DIR}/cyber/tools/cyber_channel/cyber_channel)
# file(RENAME ${CMAKE_BINARY_DIR}/cyber/tools/cyber_launch/cyber_launch.py ${CMAKE_BINARY_DIR}/cyber/tools/cyber_launch/cyber_launch)
# file(RENAME ${CMAKE_BINARY_DIR}/cyber/tools/cyber_node/cyber_node.py ${CMAKE_BINARY_DIR}/cyber/tools/cyber_node/cyber_node)
# file(RENAME ${CMAKE_BINARY_DIR}/cyber/tools/cyber_service/cyber_service.py ${CMAKE_BINARY_DIR}/cyber/tools/cyber_service/cyber_service)

# install(FILES
#   ${CMAKE_BINARY_DIR}/cyber/tools/cyber_channel/cyber_channel
#   ${CMAKE_BINARY_DIR}/cyber/tools/cyber_launch/cyber_launch
#   ${CMAKE_BINARY_DIR}/cyber/tools/cyber_node/cyber_node
#   ${CMAKE_BINARY_DIR}/cyber/tools/cyber_service/cyber_service
#   ${CMAKE_BINARY_DIR}/cyber/tools/cyber_monitor/cyber_monitor
#   ${CMAKE_BINARY_DIR}/cyber/tools/cyber_recorder/cyber_recorder
#   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#   DESTINATION bin/cyber/tools
# )

install(PROGRAMS ${CMAKE_SOURCE_DIR}/cyber/tools/cyber_channel/cyber_channel.py
        DESTINATION bin/cyber/tools
        RENAME cyber_channel)

install(PROGRAMS ${CMAKE_SOURCE_DIR}/cyber/tools/cyber_launch/cyber_launch.py
        DESTINATION bin/cyber/tools
        RENAME cyber_launch)

install(PROGRAMS ${CMAKE_SOURCE_DIR}/cyber/tools/cyber_node/cyber_node.py
        DESTINATION bin/cyber/tools
        RENAME cyber_node)

install(PROGRAMS ${CMAKE_SOURCE_DIR}/cyber/tools/cyber_service/cyber_service.py
        DESTINATION bin/cyber/tools
        RENAME cyber_service)

install(FILES
  ${CMAKE_SOURCE_DIR}/cyber/tools/cyber_tools_auto_complete.bash
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  DESTINATION bin/cyber/tools
)

install(TARGETS cyber_monitor cyber_recorder
  LIBRARY DESTINATION lib/cyber/tools
  ARCHIVE DESTINATION lib/cyber/tools
  RUNTIME DESTINATION bin/cyber/tools
)
