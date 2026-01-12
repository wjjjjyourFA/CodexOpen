# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_service_discovery)

add_subdirectory(role)
add_subdirectory(container)
add_subdirectory(specific_manager)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  # ${CMAKE_SOURCE_DIR}/cyber/message/protobuf_factory.cc
  ${CMAKE_SOURCE_DIR}/cyber/transport/common/identity.cc
  ${CMAKE_SOURCE_DIR}/cyber/transport/message/message_info.cc
  ${CMAKE_SOURCE_DIR}/cyber/transport/rtps/underlay_message.cc
  ${CMAKE_SOURCE_DIR}/cyber/service_discovery/role/role.cc
)

file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")
list(APPEND SRCS ${SWAP_SRC})

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
      
  # 创建 可执行文件
  add_executable(topology_manager_test topology_manager_test.cc ${SOURCES} ${SRCS})
  target_link_libraries(topology_manager_test PRIVATE GTest::GTest GTest::Main gflags glog uuid
    cyber_message cyber_proto protobuf 
    cyber_time cyber_croutine cyber_scheduler atomic
    brpc dl ssl rt crypto fastrtps fastcdr cyber_transport
  )

  # 添加测试到 CTest
  add_test(NAME TopologyManagerTest COMMAND topology_manager_test)
endif()

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 安装头文件（如果你希望其他模块使用这个库）
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/cyber/service_discovery
#         FILES_MATCHING PATTERN "*.h")