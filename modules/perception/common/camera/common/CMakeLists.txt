# modules/perception/common/camera/common/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(perception_common_camera_common)

file(GLOB SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/undistortion_handler.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/undistortion_handler_cv.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()

  add_executable(undistortion_handler_test
    undistortion_handler_test.cpp
    undistortion_handler.cc
    undistortion_handler_cv.cc
    ${SOURCES}
  )
  target_link_libraries(undistortion_handler_test GTest::GTest GTest::Main glog
    cyber_binary 
    perception_common_base perception_common_camera_params 
    ${OpenCV_LIBS}
  )

  add_test(NAME UndistortionHandlerTest COMMAND undistortion_handler_test)
endif()

add_library(${PROJECT_NAME} STATIC
  ${SOURCES}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    perception_common_base
    perception_common_camera_params
)

install_module(${PROJECT_NAME} modules/perception/common/camera/common)
