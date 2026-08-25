cmake_minimum_required(VERSION 3.16)

project(perception_common_algorithm_image_processing)

add_library(${PROJECT_NAME} STATIC
  hough_transfer.cc
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(BUILD_TESTING)
  add_executable(hough_transfer_test
    hough_transfer_test.cc
  )
  target_link_libraries(hough_transfer_test PRIVATE
    ${PROJECT_NAME}
    GTest::GTest
    GTest::Main
  )
  add_test(NAME HoughTransferTest COMMAND hough_transfer_test)
endif()

install_module(${PROJECT_NAME}
  modules/perception/common/algorithm/image_processing)
