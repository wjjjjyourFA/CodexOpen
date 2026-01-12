
project(cyber_examples_timer_component_example VERSION 1.0.0)

add_library(timer_component_example SHARED
  timer_component_example.cc
  # ../proto/examples.pb.cc
)

target_link_libraries(timer_component_example
  ${DEPENCENCY_LIBS} 
  cyber
)

set_target_properties(timer_component_example PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/examples/timer_component_example/"
)

# file(COPY timer.dag timer.launch
#   DESTINATION ${CMAKE_BINARY_DIR}/cyber/examples/timer_component_example
# )
install(FILES timer.dag timer.launch
  DESTINATION bin/cyber/examples/timer_component_example
)

install(TARGETS timer_component_example
  LIBRARY DESTINATION lib/cyber/examples/timer_component_example
)

