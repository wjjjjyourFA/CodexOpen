# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_blocker)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
    
  # 创建 可执行文件
  add_executable(blocker_test blocker_test.cc)
  target_link_libraries(blocker_test PRIVATE GTest::GTest GTest::Main 
    cyber_proto protobuf
    glog
  )

  add_executable(blocker_manager_test blocker_manager_test.cc blocker_manager.cc
    ${CMAKE_SOURCE_DIR}/cyber/statistics/statistics.cc
    ${CMAKE_SOURCE_DIR}/cyber/event/perf_event_cache.cc
    ${CMAKE_SOURCE_DIR}/cyber/task/task_manager.cc
    # ${CMAKE_SOURCE_DIR}/cyber/message/protobuf_factory.cc
    # ${CMAKE_SOURCE_DIR}/cyber/service_discovery/specific_manager/channel_manager.cc
    ${SOURCES}
  )
  target_link_libraries(blocker_manager_test PRIVATE GTest::GTest GTest::Main 
    cyber_service_discovery cyber_message cyber_proto protobuf cyber_time 
    cyber_croutine cyber_scheduler atomic
    cyber_transport 
    brpc dl ssl rt crypto
    fastrtps fastcdr
    gflags glog uuid
  )

  # 添加测试到 CTest
  add_test(NAME BlockerTest COMMAND blocker_test)
  add_test(NAME BlockerManagerTest COMMAND blocker_manager_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")
# 添加新的
list(APPEND SRCS ${SWAP_SRC})

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 安装头文件（如果你希望其他模块使用这个库）
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/cyber/blocker
#         FILES_MATCHING PATTERN "*.h")