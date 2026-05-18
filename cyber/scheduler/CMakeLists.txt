# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_scheduler)

add_subdirectory(common)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/cyber_time.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/duration.cc
  ${CMAKE_SOURCE_DIR}/cyber/time/clock.cc
  ${CMAKE_SOURCE_DIR}/cyber/croutine/croutine.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/scheduler_factory.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/processor.cc 
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_context.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/common/pin_thread.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/policy/choreography_context.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/policy/scheduler_classic.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/policy/scheduler_choreography.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/policy/classic_context.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/scheduler.cc 
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

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
      
  # 随着代码越多，需要添加更多的源文件，需要改用 lib 方式
  add_executable(processor_test processor_test.cc 
    ${SOURCES}
  )
  target_link_libraries(processor_test PRIVATE GTest::GTest GTest::Main
    brpc dl ssl rt crypto
    cyber_proto protobuf cyber_croutine atomic 
    fastrtps fastcdr
    gflags glog uuid
  )

  add_executable(scheduler_test scheduler_test.cc 
    ${SOURCES} ${INIT_SRCS}
  )
  target_link_libraries(scheduler_test PRIVATE GTest::GTest GTest::Main 
    brpc dl ssl rt crypto
    cyber_proto protobuf cyber_croutine atomic
    ${INIT_LIBS}
    gflags glog uuid
  )

  add_executable(scheduler_choreo_test scheduler_choreo_test.cc 
    ${SOURCES} ${INIT_SRCS})
  target_link_libraries(scheduler_choreo_test PRIVATE GTest::GTest GTest::Main
    brpc dl ssl rt crypto
    cyber_proto protobuf cyber_croutine atomic 
    ${INIT_LIBS}
    gflags glog uuid
  )

  add_executable(scheduler_classic_test scheduler_classic_test.cc 
    ${SOURCES} ${INIT_SRCS})
  target_link_libraries(scheduler_classic_test PRIVATE GTest::GTest GTest::Main
    brpc dl ssl rt crypto
    cyber_proto protobuf cyber_croutine atomic 
    ${INIT_LIBS}
    gflags glog uuid
  )

  # 添加测试到 CTest
  add_test(NAME ProcessorTest COMMAND processor_test)
  add_test(NAME SchedulerTest COMMAND scheduler_test)
  add_test(NAME SchedulerChoreoTest COMMAND scheduler_choreo_test)
  add_test(NAME SchedulerClassicTest COMMAND scheduler_classic_test)
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
#         DESTINATION include/cyber/scheduler
#         FILES_MATCHING PATTERN "*.h")