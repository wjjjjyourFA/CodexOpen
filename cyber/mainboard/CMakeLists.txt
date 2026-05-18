cmake_minimum_required(VERSION 3.5)

project(mainboard VERSION 1.0.0)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GperfTools_INCLUDE_DIRS}
)
link_directories(${GperfTools_LIBRARY_DIRS})

add_executable(mainboard
  mainboard.cc
  module_argument.cc
  module_controller.cc
)

target_link_libraries(mainboard
  ${DEPENCENCY_LIBS} 
  cyber
  # cyber_proto
  ${GperfTools_LIBRARIES} 
  # pthread
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/cyber/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/cyber/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/cyber/${PROJECT_NAME}
  RUNTIME DESTINATION bin/cyber/${PROJECT_NAME}
)
