# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_data_fusion)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  # ${CMAKE_SOURCE_DIR}/cyber/node/node.cc
  # ${CMAKE_SOURCE_DIR}/cyber/blocker/blocker_manager.cc
  # ${CMAKE_SOURCE_DIR}/cyber/service_discovery/specific_manager/channel_manager.cc
  # ${CMAKE_SOURCE_DIR}/cyber/service_discovery/specific_manager/manager.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
    
  add_executable(all_latest_test all_latest_test.cc ${SOURCES})
  target_link_libraries(all_latest_test PRIVATE GTest::GTest GTest::Main 
    brpc dl ssl rt crypto
    cyber_proto protobuf 
    # cyber_time cyber_croutine cyber_scheduler
    # fastrtps fastcdr
    gflags glog uuid
  )

  # 添加测试到 CTest
  add_test(NAME AllLatestTest COMMAND all_latest_test)
endif()
