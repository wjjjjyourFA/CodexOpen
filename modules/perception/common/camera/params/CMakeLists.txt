# modules/perception/common/camera/params/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(perception_common_camera_params)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

# include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${OPENCV_INCLUDE_DIR})
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(${CMAKE_SOURCE_DIR}/cyber)

file(GLOB SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/camera_params.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/camera_params_json.cpp"
)

add_library(${PROJECT_NAME} STATIC
  ${SOURCES}
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
    perception_common_config 
    opencv_core
    opencv_imgproc
  PRIVATE nlohmann_json::nlohmann_json
)

install_module(${PROJECT_NAME} modules/perception/common/camera/params)
