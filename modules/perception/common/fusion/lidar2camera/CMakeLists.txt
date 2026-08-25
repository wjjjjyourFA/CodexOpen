# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(perception_common_fusion_lidar2camera)

set(CMAKE_VERBOSE_MAKEFILE OFF)

find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
# include_directories(${OPENCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)

# 包含源代码目录
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(${CMAKE_SOURCE_DIR})
# include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
set(APOLLO_SOURCES
  # ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/base/camera.cc
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/base/distortion_model.cc
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/base/fisheye_model.cc
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/config/utils.cpp
)
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)

set(CODEX_SOURCES
  # ${CMAKE_SOURCE_DIR}/modules/common/math/math_utils_extra.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/common/undistortion_handler.cc
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/common/undistortion_handler_cv.cc
  # ${CMAKE_SOURCE_DIR}/modules/perception/common/camera/params/camera_params.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/tools/opencv/cv_colors.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/tools/opencv/colors.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_2d.cpp
  # ${CMAKE_SOURCE_DIR}/modules/perception/tools/common/show_data_3d.cpp
)

file(GLOB SOURCES
  # "${CMAKE_CURRENT_SOURCE_DIR}/lidar_camera_fusion_test.cpp"
  # "${CMAKE_CURRENT_SOURCE_DIR}/ros1_convert.cpp"
  # "${CMAKE_CURRENT_SOURCE_DIR}/ros2_convert.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/lidar_camera_fusion.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/lidar_multi_camera_fusion.cpp"
  # "${CMAKE_CURRENT_SOURCE_DIR}/config/runtime_config.cpp"
  # "${CMAKE_CURRENT_SOURCE_DIR}/config/runtime_config_realtime.cpp"
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING AND EXISTS
   "${CMAKE_CURRENT_SOURCE_DIR}/lidar_camera_fusion_test.cpp")
  add_executable(lidar_camera_fusion_test
    lidar_camera_fusion_test.cpp
    ${APOLLO_SOURCES}
    ${CODEX_SOURCES}
    ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/config/runtime_config.cpp
  )

  target_link_libraries(lidar_camera_fusion_test PRIVATE 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    glog
    ${Boost_LIBRARIES} boost_thread
    cyber_binary
    common_math
    perception_common_base
    perception_common_config
    perception_common_camera
    perception_tools
    perception_common_lidar_convert
  )

  if(OpenMP_CXX_FOUND)
    target_link_libraries(lidar_camera_fusion_test PRIVATE OpenMP::OpenMP_CXX)
  endif()
endif()

if(BUILD_TESTING)
  add_executable(lidar_camera_fusion_core_test
    lidar_camera_fusion_core_test.cpp
  )
  target_link_libraries(lidar_camera_fusion_core_test PRIVATE
    ${PROJECT_NAME}
  )
  add_test(NAME LidarCameraFusionCoreTest
           COMMAND lidar_camera_fusion_core_test)
endif()

option(BUILD_ROS1 "Enable ROS1" OFF)

if(BUILD_ROS1)
  # list(APPEND SOURCES
  #   "${CMAKE_CURRENT_SOURCE_DIR}/ros1_convert.cpp"
  #   "${CMAKE_CURRENT_SOURCE_DIR}/config/runtime_config_realtime.cpp"
  # )

  add_executable(lidar_camera_fusion_realtime_ros1
    run_lidar_camera_fusion_realtime.cpp
    ${APOLLO_SOURCES}
    ${CODEX_SOURCES}
    ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_convert.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/config/runtime_config.cpp
    # ${CMAKE_SOURCE_DIR}/modules/perception/common/lidar/convert/robosense.cpp
    # ${CMAKE_SOURCE_DIR}/modules/perception/common/lidar/convert/velodyne.cpp
  )

  target_include_directories(lidar_camera_fusion_realtime_ros1 PUBLIC 
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${ROS1_INCLUDE_DIRS}
  )

  target_link_libraries(lidar_camera_fusion_realtime_ros1 PRIVATE 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    glog
    ${Boost_LIBRARIES} boost_thread
    ${ROS1_LIBRARIES}
    cyber_binary
    common_math
    perception_common_base
    perception_common_config
    perception_common_camera
    perception_tools
    perception_common_lidar_convert
    # perception_common_lidar
  )

  if(OpenMP_CXX_FOUND)
    target_link_libraries(lidar_camera_fusion_realtime_ros1 PRIVATE OpenMP::OpenMP_CXX)
  endif()

  # 安装到自定义目录
  install_module(lidar_camera_fusion_realtime_ros1 modules/perception/common/fusion/lidar2camera)
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
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    glog
    ${Boost_LIBRARIES} boost_thread
    cyber_binary
    common_math
    perception_common_base
    perception_common_config
    perception_common_camera
    perception_tools
    perception_common_lidar_convert
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

install_module(${PROJECT_NAME} modules/perception/common/fusion/lidar2camera)
