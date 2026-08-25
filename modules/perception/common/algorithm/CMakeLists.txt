cmake_minimum_required(VERSION 3.16)

project(perception_common_algorithm)

add_subdirectory(image_processing)
add_subdirectory(point_cloud_processing)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  perception_common_algorithm_image_processing
  perception_common_algorithm_point_cloud_processing
)

if(BUILD_TESTING)
  add_executable(algorithm_core_test algorithm_core_test.cpp)
  target_link_libraries(algorithm_core_test PRIVATE
    ${PROJECT_NAME}
    perception_common_algorithm_image_processing
    perception_common_algorithm_point_cloud_processing
  )
  add_test(NAME PerceptionAlgorithmCoreTest COMMAND algorithm_core_test)
endif()

install_module(${PROJECT_NAME} modules/perception/common/algorithm)