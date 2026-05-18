# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_scheduler_common)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
      
  add_executable(pin_thread_test pin_thread_test.cc
    pin_thread.cc ${SOURCES}
    # ${CMAKE_SOURCE_DIR}/cyber/node/node.cc
    # ${CMAKE_SOURCE_DIR}/cyber/init.cc
    # ${CMAKE_SOURCE_DIR}/cyber/state.cc
  )
  target_link_libraries(pin_thread_test PRIVATE GTest::GTest GTest::Main gflags glog
    brpc dl ssl rt crypto 
    cyber_proto protobuf 
    # cyber_time cyber_croutine cyber_scheduler cyber_component 
    # fastrtps fastcdr
  )

  # 添加测试到 CTest
  add_test(NAME PinThreadTest COMMAND pin_thread_test)
endif()