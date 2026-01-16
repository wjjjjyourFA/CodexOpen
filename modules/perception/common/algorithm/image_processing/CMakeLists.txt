# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(perception_common_algorithm_image_processing)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR})

add_executable(hough_transfer_test
    ${CMAKE_CURRENT_SOURCE_DIR}/hough_transfer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/hough_transfer_test.cc
)
target_link_libraries(hough_transfer_test GTest::GTest GTest::Main glog)

enable_testing()

add_test(NAME HoughTransferTest COMMAND hough_transfer_test)