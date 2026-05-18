cmake_minimum_required(VERSION 3.5)

project(cyber_examples VERSION 1.0.0)

# add_subdirectory(proto)

add_executable(talker talker.cc 
  # proto/examples.pb.cc
)
target_link_libraries(talker
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
  # brpc
  pthread
)

add_executable(listener listener.cc 
  # proto/examples.pb.cc
)
target_link_libraries(listener
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
  # brpc
  pthread
)

add_executable(paramserver paramserver.cc)
target_link_libraries(paramserver
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
)

add_executable(record record.cc)
target_link_libraries(record
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
)

add_executable(service service.cc 
  # proto/examples.pb.cc
)
target_link_libraries(service
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
)

add_subdirectory(common_component_example)
add_subdirectory(timer_component_example)

install(TARGETS 
  talker listener 
  paramserver record
  service
  LIBRARY DESTINATION lib/cyber/examples
  ARCHIVE DESTINATION lib/cyber/examples
  RUNTIME DESTINATION bin/cyber/examples
)