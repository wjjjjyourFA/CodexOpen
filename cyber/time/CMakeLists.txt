# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_time)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/cyber_time.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/duration.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/clock.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()

  add_executable(clock_test clock_test.cc ${SOURCES})
  target_link_libraries(clock_test PRIVATE GTest::GTest GTest::Main glog 
    cyber_proto protobuf 
  )

  add_executable(duration_test duration_test.cc ${SOURCES})
  target_link_libraries(duration_test PRIVATE GTest::GTest GTest::Main glog    
    cyber_proto protobuf
  )

  add_executable(time_test time_test.cc ${SOURCES})
  target_link_libraries(time_test PRIVATE GTest::GTest GTest::Main glog
    cyber_proto protobuf
  )
  # target_link_libraries(time_test pthread)  # 如果使用 pthread
  # target_link_libraries(time_test rt)       # 如果需要支持实时库

  enable_testing()

  # 添加测试到 CTest
  add_test(NAME ClockTest COMMAND clock_test)
  add_test(NAME DurationTest COMMAND duration_test)
  add_test(NAME TimeTest COMMAND time_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 安装头文件（如果你希望其他模块使用这个库）
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/cyber/time
#         FILES_MATCHING PATTERN "*.h")