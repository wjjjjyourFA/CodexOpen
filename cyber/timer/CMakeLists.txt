# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_timer)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cyber)

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/task/task_manager.cc
)

set(INIT_SRCS
  ${CMAKE_SOURCE_DIR}/cyber/init.cc
  ${CMAKE_SOURCE_DIR}/cyber/sysmo/sysmo.cc
  ${CMAKE_SOURCE_DIR}/cyber/timer/timing_wheel.cc
  ${CMAKE_SOURCE_DIR}/cyber/statistics/statistics.cc
  ${CMAKE_SOURCE_DIR}/cyber/event/perf_event_cache.cc
  ${CMAKE_SOURCE_DIR}/cyber/task/task_manager.cc
)

set(INIT_LIBS
  cyber_service_discovery cyber_message cyber_proto protobuf 
  cyber_time atomic
  cyber_node cyber_transport cyber_blocker cyber_logger
  brpc dl ssl rt crypto 
  fastrtps fastcdr
)

file(GLOB_RECURSE SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()

  add_executable(timer_test timer_test.cc ${SOURCES} ${SRCS} ${INIT_SRCS}
    ${CMAKE_SOURCE_DIR}/cyber/sysmo/sysmo.cc
    ${CMAKE_SOURCE_DIR}/cyber/timer/timing_wheel.cc
  )
  target_link_libraries(timer_test PRIVATE GTest::GTest GTest::Main
    brpc dl ssl rt crypto 
    cyber_proto protobuf 
    cyber_time cyber_croutine cyber_scheduler atomic
    fastrtps fastcdr
    ${INIT_LIBS}
    gflags glog uuid
  )

  # 添加测试到 CTest
  add_test(NAME TimerTest COMMAND timer_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

if(SRCS)
  add_library(${PROJECT_NAME} STATIC ${SRCS})
  target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# 安装头文件（如果你希望其他模块使用这个库）
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/cyber/timer
#         FILES_MATCHING PATTERN "*.h")