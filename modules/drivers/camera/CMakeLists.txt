# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(drivers_camera)

# 启用 AVX2 指令集优化
# 检查是否为非 ARM 架构（即 x86 或 x86_64 等）
if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "arm" AND NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Detected x86 architecture, enabling -mavx2")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
else()
  message(STATUS "ARM architecture detected, skipping -mavx2")
endif()

add_subdirectory(proto)

find_package(Protobuf REQUIRED 3.6)
# message(STATUS "===== Protobuf CMake Info =====")
# message(STATUS "Protobuf version: ${Protobuf_VERSION}")
# message(STATUS "Protobuf include dirs: ${Protobuf_INCLUDE_DIRS}")
# message(STATUS "Protobuf libraries: ${Protobuf_LIBRARIES}")
# message(STATUS "Protobuf protoc executable: ${Protobuf_PROTOC_EXECUTABLE}")
# message(STATUS "================================")

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/cyber_time.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/duration.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/config.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_cam.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/util.cc
)
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)

if(BUILD_TESTING)
  add_executable(usb_cam_test
      usb_cam_test.cpp
      ${SOURCES}
  )

  target_link_libraries(usb_cam_test PRIVATE 
    # ${PCL_LIBRARIES}
    # pcl_common
    # pcl_io
    # pcl_filters
    # ${OpenCV_LIBRARIES}
    glog 
    # ==> conflict in /usr/lib
    # protobuf 
    # ==> from find_package(Protobuf REQUIRED 3.6)
    ${Protobuf_LIBRARIES} 
    # /usr/local/lib/libprotobuf.so
    # ${Boost_LIBRARIES} boost_thread
    avcodec avutil swscale avformat
  )
endif()

# add_subdirectory(kdtree)