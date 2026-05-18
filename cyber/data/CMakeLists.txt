# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_data)

# 包含源代码目录
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(SOURCES
  # ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  # ${CMAKE_SOURCE_DIR}/cyber/state.cc
  # ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  # ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  # ${CMAKE_SOURCE_DIR}/cyber/node/node.cc
  # ${CMAKE_SOURCE_DIR}/cyber/init.cc
  # ${CMAKE_SOURCE_DIR}/cyber/sysmo/sysmo.cc
  # ${CMAKE_SOURCE_DIR}/cyber/timer/timing_wheel.cc
  # ${CMAKE_SOURCE_DIR}/cyber/blocker/blocker_manager.cc
  # ${CMAKE_SOURCE_DIR}/cyber/service_discovery/specific_manager/channel_manager.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
      
  add_executable(cache_buffer_test cache_buffer_test.cc)
  target_link_libraries(cache_buffer_test GTest::GTest GTest::Main glog
    cyber_base cyber_common
  )

  add_executable(data_dispatcher_test data_dispatcher_test.cc ${SOURCES})
  target_link_libraries(data_dispatcher_test PRIVATE GTest::GTest GTest::Main   
    # brpc dl ssl rt crypto
    cyber_proto protobuf 
    cyber_base cyber_common cyber_state
    # cyber_service_discovery
    # fastrtps fastcdr
    gflags glog uuid
  )

  add_executable(data_visitor_test data_visitor_test.cc ${SOURCES})
  target_link_libraries(data_visitor_test PRIVATE GTest::GTest GTest::Main
    # brpc dl ssl rt crypto
    cyber_proto protobuf 
    cyber_base cyber_common cyber_state
    # cyber_scheduler
    # fastrtps fastcdr
    gflags glog uuid
  )

  add_executable(channel_buffer_test channel_buffer_test.cc ${SOURCES})
  target_link_libraries(channel_buffer_test PRIVATE GTest::GTest GTest::Main
    # brpc dl ssl rt crypto
    cyber_proto protobuf 
    cyber_base cyber_common cyber_state
    # cyber_service_discovery
    # fastrtps fastcdr
    gflags glog uuid
  )

  # 添加测试到 CTest
  add_test(NAME CacheBufferTest COMMAND cache_buffer_test)
  add_test(NAME DataDispatcherTest COMMAND data_dispatcher_test)
  add_test(NAME DataVisitorTest COMMAND data_visitor_test)
  add_test(NAME ChannelBufferTest COMMAND channel_buffer_test)
endif()

add_subdirectory(fusion)

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

if(SRCS)
  add_library(${PROJECT_NAME} STATIC ${SRCS})
  target_include_directories(${PROJECT_NAME} 
    PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(${PROJECT_NAME} PUBLIC cyber_base cyber_common cyber_state)
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} 
    INTERFACE 
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE cyber_base cyber_common cyber_state)
endif()

install_module(${PROJECT_NAME} cyber/data)