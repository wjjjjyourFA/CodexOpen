# modules/perception/common/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(perception_common_base)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})

# 包含源代码目录
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(${CMAKE_SOURCE_DIR}/cyber)
# include_directories(${CMAKE_SOURCE_DIR}/install/third_party/include)

# 添加源文件
set(SOURCES
  # ${CMAKE_SOURCE_DIR}/cyber/binary.cc
)

if(BUILD_TESTING)
  enable_testing()

  add_executable(camera_test
    camera_test.cc
    camera.cc
    ${SOURCES}
  )
  target_link_libraries(camera_test GTest::GTest GTest::Main glog)

  add_executable(distortion_model_test
    distortion_model_test.cc
    distortion_model.cc
    camera.cc
    ${SOURCES}
  )
  target_link_libraries(distortion_model_test GTest::GTest GTest::Main glog 
    ${OpenCV_LIBS}
    cyber_binary 
  )

  add_executable(polynomial_test
    polynomial_test.cc
    polynomial.cc
    ${SOURCES}
  )
  target_link_libraries(polynomial_test GTest::GTest GTest::Main glog ${OpenCV_LIBS})

  add_executable(omnidirectional_model_test
    omnidirectional_model_test.cc
    omnidirectional_model.cc
    camera.cc
    distortion_model.cc
    polynomial.cc
    ${SOURCES}
  )
  target_link_libraries(omnidirectional_model_test GTest::GTest GTest::Main glog 
    ${OpenCV_LIBS}
    cyber_binary 
  )

  add_executable(box_test
    box_test.cc
    ${SOURCES}
  )
  target_link_libraries(box_test GTest::GTest GTest::Main glog)

  add_executable(box3d_test
    box3d_test.cc
    ${SOURCES}
  )
  target_link_libraries(box3d_test GTest::GTest GTest::Main glog)

  # add_executable(perception_common_test
  #   common_test.cc
  #   ${SOURCES}
  # )
  # target_link_libraries(perception_common_test GTest::GTest GTest::Main glog)

  # add_executable(syncedmem_test
  #   syncedmem_test.cc
  #   syncedmem.cc
  #   test/test_helper.h
  #   ${SOURCES}
  # )
  # target_link_libraries(syncedmem_test GTest::GTest GTest::Main glog)

  # add_executable(blob_test
  #   blob_test.cc
  #   blob.cc
  #   syncedmem.cc
  #   ${SOURCES}
  # )
  # target_include_directories(blob_test PRIVATE)
  # target_link_libraries(blob_test GTest::GTest GTest::Main glog)

  # add_executable(image8u_test
  #   image_test.cc
  #   blob.cc
  #   syncedmem.cc
  #   ${SOURCES}
  # )
  # target_include_directories(image8u_test PRIVATE)
  # target_link_libraries(image8u_test GTest::GTest GTest::Main glog)

  # add_executable(impending_collision_edge_test
  #   impending_collision_edge_test.cc
  #   ${SOURCES}
  # )
  # target_link_libraries(impending_collision_edge_test GTest::GTest GTest::Main glog)

  # add_executable(point_cloud_test
  #   point_cloud_test.cc
  #   ${SOURCES}
  # )
  # target_link_libraries(point_cloud_test GTest::GTest GTest::Main glog)

  # 添加测试到 CTest
  add_test(NAME CameraTest COMMAND camera_test)
  add_test(NAME DistortionModelTest COMMAND distortion_model_test)
  add_test(NAME PolynomialTest COMMAND polynomial_test)
  add_test(NAME OmnidirectionalModelTest COMMAND omnidirectional_model_test)
  add_test(NAME BoxTest COMMAND box_test)
  add_test(NAME Box3dTest COMMAND box3d_test)
  # add_test(NAME PerceptionCommonTest COMMAND perception_common_test)
  # add_test(NAME SyncedmemTest COMMAND syncedmem_test)
  # add_test(NAME BlobTest COMMAND blob_test)
  # add_test(NAME ImageTest COMMAND image8u_test)
  # add_test(NAME ImpendingCollisionEdgeTest COMMAND impending_collision_edge_test)
  # add_test(NAME PointCloudTest COMMAND point_cloud_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

# 合并指定源文件和自动收集的源文件（避免重复）
list(APPEND SRCS ${SOURCES})
list(REMOVE_DUPLICATES SRCS)

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    cyber_binary
    opencv_core
    opencv_imgproc
    ${PCL_LIBRARIES}
    glog
)

install_module(${PROJECT_NAME} modules/perception/common/base)