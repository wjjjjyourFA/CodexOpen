cmake_minimum_required(VERSION 3.16)

project(common_transform)

set(SRCS
  adapters/isam_adapter.cpp
  adapters/loam_adapter.cpp
  geometry/rotation_conversions.cpp
)

add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    common_struct
)

if(BUILD_TESTING)
  add_executable(transform_adapters_test transform_adapters_test.cc)
  target_link_libraries(transform_adapters_test
    PRIVATE
      ${PROJECT_NAME}
      GTest::GTest
      GTest::Main
  )
  add_test(NAME TransformAdaptersTest COMMAND transform_adapters_test)
endif()

install_module(${PROJECT_NAME} modules/common/transform)
