# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(common_math)

# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(OpenCV REQUIRED)
# include_directories(${OPENCV_INCLUDE_DIR})

# 包含源代码目录
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
set(SOURCES
  # ${CMAKE_SOURCE_DIR}/cyber/binary.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()

  add_executable(angle_test
    angle_test.cc
    angle.cc
    sin_table.cc
    ${SOURCES}
  )
  target_link_libraries(angle_test GTest::GTest GTest::Main glog
    cyber_binary
  )

  add_executable(vec2d_test
    vec2d_test.cc
    vec2d.cc
    ${SOURCES}
  )
  target_include_directories(vec2d_test PRIVATE)
  target_link_libraries(vec2d_test GTest::GTest GTest::Main glog 
    absl::strings_internal absl::strings absl::base
    cyber_binary
  )

  add_executable(math_utils_test
    math_utils_test.cc
    math_utils.cc
    vec2d.cc
    ${SOURCES}
  )
  target_include_directories(math_utils_test PRIVATE)
  target_link_libraries(math_utils_test GTest::GTest GTest::Main glog 
    absl::base absl::strings osqp
    cyber_binary
  )

  add_executable(matrix_operations_test
    matrix_operations_test.cc
    matrix_operations.cc
    ${SOURCES}
  )
  target_link_libraries(matrix_operations_test GTest::GTest GTest::Main glog
    cyber_binary
  )

  add_executable(kalman_filter_test
    kalman_filter_test.cc
    matrix_operations.cc
    ${SOURCES}
  )
  target_link_libraries(kalman_filter_test GTest::GTest GTest::Main glog
    cyber_binary
  )

  add_executable(euler_angles_xyz_test
    euler_angles_xyz_test.cc
    math_utils.cc
    vec2d.cc
    ${SOURCES}
  )
  target_link_libraries(euler_angles_xyz_test GTest::GTest GTest::Main glog
    absl_base absl_strings
    cyber_binary
  )

  add_executable(quaternion_test
    quaternion_test.cc
    math_utils.cc
    vec2d.cc
    ${SOURCES}
  )
  target_link_libraries(quaternion_test GTest::GTest GTest::Main glog
    absl_base absl_strings
    cyber_binary
  )

  add_executable(aabox2d_test
    aabox2d_test.cc
    aabox2d.cc
    math_utils.cc
    vec2d.cc
    ${SOURCES}
  )
  target_link_libraries(aabox2d_test GTest::GTest GTest::Main glog
    absl_base absl_strings
    cyber_binary
  )

  add_executable(line_segment2d_test
    line_segment2d_test.cc
    line_segment2d.cc
    math_utils.cc
    vec2d.cc
    ${SOURCES}
  )
  target_link_libraries(line_segment2d_test GTest::GTest GTest::Main glog
    absl_base absl_strings
    cyber_binary
  )

  add_executable(polygon2d_test
    polygon2d_test.cc
    polygon2d.cc
    line_segment2d.cc
    box2d.cc
    aabox2d.cc
    math_utils.cc
    vec2d.cc
    ${SOURCES}
  )
  target_link_libraries(polygon2d_test GTest::GTest GTest::Main glog
    absl_base absl_strings
    cyber_binary common_util
  )

  # 添加测试到 CTest
  add_test(NAME AngleTest COMMAND angle_test)
  add_test(NAME MathUtilsTest COMMAND math_utils_test)
  add_test(NAME Vec2dTest COMMAND vec2d_test)
  add_test(NAME MatrixOperationsTest COMMAND matrix_operations_test)
  add_test(NAME KalmanFilterTest COMMAND kalman_filter_test)
  add_test(NAME EulerAnglesXyzTest COMMAND euler_angles_xyz_test)
  add_test(NAME QuaternionTest COMMAND quaternion_test)
  add_test(NAME Aabox2dTest COMMAND aabox2d_test)
  add_test(NAME LineSegment2dTest COMMAND line_segment2d_test)
  add_test(NAME Polygon2dTest COMMAND polygon2d_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

# 合并指定源文件和自动收集的源文件（避免重复）
list(APPEND ALL_SRCS ${SOURCES})
list(REMOVE_DUPLICATES ALL_SRCS)

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC cyber_binary common_util)

install_module(${PROJECT_NAME} modules/common/math)