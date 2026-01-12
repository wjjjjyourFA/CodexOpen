# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(common_math)

# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(OpenCV REQUIRED)
# include_directories(${OPENCV_INCLUDE_DIR})

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
# set(SOURCES
#     ${CMAKE_SOURCE_DIR}/cyber/binary.cc
# )

if(BUILD_TESTING)
  add_executable(angle_test
      angle_test.cc
      angle.cc
      sin_table.cc
      ${SOURCES}
  )
  target_link_libraries(angle_test GTest::GTest GTest::Main glog)

  add_executable(vec2d_test
      vec2d_test.cc
      vec2d.cc
      ${SOURCES}
  )
  target_include_directories(vec2d_test PRIVATE)
  target_link_libraries(vec2d_test GTest::GTest GTest::Main glog 
      # absl_base absl_strings
  )
  # target_link_directories(vec2d_test PRIVATE ${ABSEIL_INSTALL_DIR}/lib)

  add_executable(math_utils_test
      math_utils_test.cc
      math_utils.cc
      vec2d.cc
      ${SOURCES}
  )
  target_include_directories(math_utils_test PRIVATE)
  target_link_libraries(math_utils_test GTest::GTest GTest::Main glog 
      # absl_base absl_strings osqp
  )

  add_executable(matrix_operations_test
      matrix_operations_test.cc
      matrix_operations.cc
      ${SOURCES}
  )
  target_link_libraries(matrix_operations_test GTest::GTest GTest::Main glog)

  add_executable(kalman_filter_test
      kalman_filter_test.cc
      matrix_operations.cc
      ${SOURCES}
  )
  target_link_libraries(kalman_filter_test GTest::GTest GTest::Main glog)

  enable_testing()

  # 添加测试到 CTest
  add_test(NAME AngleTest COMMAND angle_test)
  add_test(NAME MathUtilsTest COMMAND math_utils_test)
  add_test(NAME Vec2dTest COMMAND vec2d_test)
  add_test(NAME MatrixOperationsTest COMMAND matrix_operations_test)
  add_test(NAME KalmanFilterTest COMMAND kalman_filter_test)

endif()