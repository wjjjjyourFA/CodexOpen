# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_node)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/cyber.cc
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/statistics/statistics.cc
  ${CMAKE_SOURCE_DIR}/cyber/event/perf_event_cache.cc
  ${CMAKE_SOURCE_DIR}/cyber/task/task_manager.cc
  # ${CMAKE_SOURCE_DIR}/cyber/message/protobuf_factory.cc
  # ${CMAKE_SOURCE_DIR}/cyber/service_discovery/specific_manager/channel_manager.cc
  ${CMAKE_SOURCE_DIR}/cyber/node/node.cc
)

set(INIT_SRCS
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/init.cc
  ${CMAKE_SOURCE_DIR}/cyber/sysmo/sysmo.cc
  ${CMAKE_SOURCE_DIR}/cyber/timer/timing_wheel.cc
  ${CMAKE_SOURCE_DIR}/cyber/statistics/statistics.cc
  ${CMAKE_SOURCE_DIR}/cyber/event/perf_event_cache.cc
  ${CMAKE_SOURCE_DIR}/cyber/task/task_manager.cc
)

set(INIT_LIBS
  cyber_service_discovery cyber_message cyber_proto protobuf 
  cyber_croutine cyber_scheduler cyber_time atomic
  cyber_node cyber_transport cyber_blocker cyber_logger
  brpc dl ssl rt crypto 
  fastrtps fastcdr
)

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")
# 添加新的
list(APPEND SRCS ${SWAP_SRC})

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
  
  # 创建 可执行文件
  add_executable(writer_test writer_test.cc ${SOURCES} ${INIT_SRCS})
  target_link_libraries(writer_test PRIVATE GTest::GTest GTest::Main
    cyber_service_discovery cyber_message cyber_proto protobuf 
    cyber_time cyber_croutine cyber_scheduler atomic
    cyber_transport  
    brpc dl ssl rt crypto 
    fastrtps fastcdr
    ${INIT_LIBS}
    gflags glog uuid
  )

  add_executable(reader_test reader_test.cc ${SOURCES} ${INIT_SRCS})
  target_link_libraries(reader_test PRIVATE GTest::GTest GTest::Main 
    cyber_service_discovery cyber_message cyber_proto protobuf 
    cyber_time cyber_croutine cyber_scheduler atomic
    cyber_transport  
    brpc dl ssl rt crypto 
    fastrtps fastcdr
    ${INIT_LIBS}
    gflags glog uuid
  )

  add_executable(node_channel_impl_test
    ${CMAKE_SOURCE_DIR}/cyber/transport/qos/qos_profile_conf.cc
    ${CMAKE_SOURCE_DIR}/cyber/binary.cc
    ${CMAKE_SOURCE_DIR}/cyber/state.cc
    ${INIT_SRCS}
  )
  target_link_libraries(node_channel_impl_test PRIVATE GTest::GTest GTest::Main
    cyber_proto protobuf 
    cyber_time cyber_croutine cyber_scheduler atomic
    cyber_transport cyber_service_discovery 
    brpc dl ssl rt crypto
    ${INIT_LIBS}
    gflags glog uuid
  )

  add_executable(writer_reader_test writer_reader_test.cc node.cc 
    ${CMAKE_SOURCE_DIR}/cyber/blocker/blocker_manager.cc
    ${SOURCES} ${INIT_SRCS}
  )
  target_link_libraries(writer_reader_test PRIVATE GTest::GTest GTest::Main 
    cyber_service_discovery cyber_message cyber_proto protobuf 
    cyber_time cyber_croutine cyber_scheduler atomic
    cyber_transport  
    brpc dl ssl rt crypto 
    fastrtps fastcdr
    ${INIT_LIBS}
    gflags glog uuid
  )
  # add_executable(writer_reader_test writer_reader_test.cc ${SOURCES} ${SRCS})
  # target_link_libraries(writer_reader_test GTest::GTest GTest::Main glog)

  add_executable(node_test node_test.cc node.cc 
    ${CMAKE_SOURCE_DIR}/cyber/blocker/blocker_manager.cc
    ${SOURCES} ${INIT_SRCS}
  )
  target_link_libraries(node_test PRIVATE GTest::GTest GTest::Main 
    cyber_service_discovery cyber_message cyber_proto protobuf 
    cyber_time cyber_croutine cyber_scheduler atomic
    cyber_transport  
    brpc dl ssl rt crypto 
    fastrtps fastcdr
    ${INIT_LIBS}
    gflags glog uuid
  )

  # 添加测试到 CTest
  add_test(NAME WriterTest COMMAND writer_test)
  add_test(NAME ReaderTest COMMAND reader_test)
  add_test(NAME node_channel_impl_test COMMAND node_channel_impl_test)
  add_test(NAME WriterReaderTest COMMAND writer_reader_test)
  add_test(NAME NodeTest COMMAND node_test)
endif()

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 安装头文件（如果你希望其他模块使用这个库）
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/cyber/node
#         FILES_MATCHING PATTERN "*.h")