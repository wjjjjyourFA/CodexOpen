# modules/drivers/camera/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(drivers_camera)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})

# 启用 AVX2 指令集优化
# 检查是否为非 ARM 架构（即 x86 或 x86_64 等）
if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "arm" AND NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Detected x86 architecture, enabling -mavx2")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
else()
  message(STATUS "ARM architecture detected, skipping -mavx2")
endif()

# 修改为使用外部的全局编译，容易导致重复链接
add_subdirectory(proto)

# 包含源代码目录
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(APOLLO_SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/cyber_time.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/duration.cc
  # ${CMAKE_CURRENT_SOURCE_DIR}/proto/config.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_cam.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/util.cc
)
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} APOLLO_SOURCES)

set(CODEX_SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  # ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  # ${CMAKE_SOURCE_DIR}/cyber/time/cyber_time.cc
  # ${CMAKE_SOURCE_DIR}/cyber/time/duration.cc
  # ${CMAKE_CURRENT_SOURCE_DIR}/proto/config.pb.cc
)
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/config CODEX_SOURCES)

file(GLOB SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/usb_cam_cv.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/ros1_convert.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/ros2_convert.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/jpeg_encode.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/util.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/config/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/config/*.cc"
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  add_executable(usb_cam_test
    usb_cam_test.cpp
    ${APOLLO_SOURCES}
  )

  target_link_libraries(usb_cam_test PRIVATE 
    glog 
    # ==> conflict in /usr/lib
    cyber_proto camera_proto
    # ==> from find_package(Protobuf REQUIRED 3.6)
    avcodec avutil swscale avformat
  )
endif()

option(BUILD_ROS1 "Enable ROS1" OFF)

if(BUILD_ROS1)
  list(REMOVE_ITEM SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/ros2_convert.cpp"
  )

  add_executable(camera_image_to_ros1
    usb_cam_cv_test.cpp
    ${CODEX_SOURCES}
    ${SOURCES}
  )

  target_include_directories(camera_image_to_ros1 PUBLIC 
    ${OpenCV_INCLUDE_DIRS}
    ${ROS1_INCLUDE_DIRS}
  )

  target_link_libraries(camera_image_to_ros1 PRIVATE 
    # ${PCL_LIBRARIES}
    # pcl_common
    # pcl_io
    # pcl_filters
    ${OpenCV_LIBRARIES}
    glog 
    # ==> conflict in /usr/lib
    cyber_proto camera_proto
    # ==> from find_package(Protobuf REQUIRED 3.6)
    # ${Protobuf_LIBRARIES} 
    # /usr/local/lib/libprotobuf.so
    # ${Boost_LIBRARIES} boost_thread
    avcodec avutil swscale avformat
    ${ROS1_LIBRARIES}
    yaml-cpp
    yuv turbojpeg
  )
  if(NOT ROS1_LIBRARIES)
    message(WARNING "ROS1_LIBRARIES is empty!")
  endif()
  # message(STATUS "ROS1_LIBRARIES = ${ROS1_LIBRARIES}")

  # 避免 RPATH 冲突，优先 /usr/local/lib ==> ${CMAKE_INSTALL_INCLUDEDIR}
  set_target_properties(camera_image_to_ros1 PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH ${CMAKE_INSTALL_INCLUDEDIR}
    SKIP_BUILD_RPATH FALSE
  )

  # 安装到自定义目录
  install_module(camera_image_to_ros1 modules/drivers/camera)
endif()