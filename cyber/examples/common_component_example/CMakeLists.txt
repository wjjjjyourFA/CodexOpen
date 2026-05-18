
project(cyber_examples_common_component_example VERSION 1.0.0)

add_executable(channel_test_writer
  channel_test_writer.cc 
  # ../proto/examples.pb.cc
)

target_link_libraries(channel_test_writer
  ${DEPENCENCY_LIBS} 
  cyber
)

add_executable(channel_prediction_writer
  channel_prediction_writer.cc 
  # ../proto/examples.pb.cc
)

target_link_libraries(channel_prediction_writer
  ${DEPENCENCY_LIBS} 
  cyber
)

add_library(common_component_example SHARED
  common_component_example.cc
  # ../proto/examples.pb.cc
)

target_link_libraries(common_component_example
  ${DEPENCENCY_LIBS} 
  cyber
)

set_target_properties(common_component_example PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/examples/common_component_example/"
)

# file(COPY common.dag common.launch
#   DESTINATION ${CMAKE_BINARY_DIR}/cyber/examples/common_component_example
# )
install(FILES common.dag common.launch
  DESTINATION bin/cyber/examples/common_component_example
)

install(TARGETS 
  channel_test_writer channel_prediction_writer
  common_component_example
  LIBRARY DESTINATION lib/cyber/examples/common_component_example
  ARCHIVE DESTINATION lib/cyber/examples/common_component_example
  RUNTIME DESTINATION bin/cyber/examples/common_component_example
)
