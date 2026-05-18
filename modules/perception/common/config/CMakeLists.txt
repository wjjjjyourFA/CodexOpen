cmake_minimum_required(VERSION 3.16)

project(perception_common_config)

add_library(${PROJECT_NAME} STATIC
  utils.cpp
  sensor_extrinsics.cpp
  sensor_extrinsics_json.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# 如果 utils.cpp 里用了 Eigen / OpenCV，再加
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PRIVATE nlohmann_json::nlohmann_json
)

install_module(${PROJECT_NAME} modules/perception/common/config)
