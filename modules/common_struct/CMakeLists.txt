cmake_minimum_required(VERSION 3.16)

project(common_struct)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

if(BUILD_TESTING)
  add_executable(common_struct_test common_struct_test.cc)
  target_link_libraries(common_struct_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::GTest
      GTest::Main
  )
  add_test(NAME CommonStructTest COMMAND common_struct_test)
endif()

install(TARGETS ${PROJECT_NAME} EXPORT common_structTargets)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/modules/common_struct
  FILES_MATCHING PATTERN "*.h"
)
