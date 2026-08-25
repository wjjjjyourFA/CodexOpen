cmake_minimum_required(VERSION 3.16)

project(perception_common_algorithm_point_cloud_processing)

add_library(${PROJECT_NAME} STATIC
  cluster_postprocess.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  perception_common_base
)

if(BUILD_TESTING)
  add_executable(dbscan_test dbscan_test.cc)
  target_compile_features(dbscan_test PRIVATE cxx_std_17)
  target_link_libraries(dbscan_test PRIVATE
    ${PROJECT_NAME}
    GTest::GTest
    GTest::Main
    glog
  )
  add_test(NAME DbscanTest COMMAND dbscan_test)
endif()

install_module(${PROJECT_NAME}
  modules/perception/common/algorithm/point_cloud_processing)
