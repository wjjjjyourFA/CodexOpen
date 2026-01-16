# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(perception_common_algorithm_point_cloud_processing)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(dbscan_test
    ${CMAKE_CURRENT_SOURCE_DIR}/dbscan_test.cc
)
target_link_libraries(dbscan_test GTest::GTest GTest::Main glog)

add_executable(kdtree_simple_test
    ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/kdtree_simple.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/kdtree_simple_test.cc
)
target_link_libraries(kdtree_simple_test GTest::GTest GTest::Main glog)

enable_testing()

add_test(NAME DbscanTest COMMAND dbscan_test)
add_test(NAME KdtreeSimpleTest COMMAND kdtree_simple_test)

