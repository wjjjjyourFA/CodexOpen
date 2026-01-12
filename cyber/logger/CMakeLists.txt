# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_logger)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  # ${CMAKE_SOURCE_DIR}/cyber/sysmo/sysmo.cc
  # ${CMAKE_SOURCE_DIR}/cyber/timer/timing_wheel.cc
  # ${CMAKE_SOURCE_DIR}/cyber/blocker/blocker_manager.cc
  # ${CMAKE_SOURCE_DIR}/cyber/service_discovery/specific_manager/channel_manager.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
    
  add_executable(logger_util_test 
    logger_util_test.cc 
    logger_util.cc
    ${SOURCES}
  )
  target_link_libraries(logger_util_test PRIVATE GTest::GTest GTest::Main glog
    cyber_proto protobuf
  )

  add_executable(log_file_object_test 
    log_file_object_test.cc 
    log_file_object.cc 
    logger_util.cc
    # ${CMAKE_SOURCE_DIR}/cyber/init.cc
    # ${CMAKE_SOURCE_DIR}/cyber/node/node.cc
    ${SOURCES}
  )
  target_link_libraries(log_file_object_test PRIVATE GTest::GTest GTest::Main
    brpc dl ssl rt crypto
    cyber_proto protobuf 
    # cyber_scheduler
    # fastrtps fastcdr 
    gflags glog uuid
  )

  # 创建 logger_test 可执行文件
  add_executable(logger_test 
    logger_test.cc 
    logger.cc
    log_file_object.cc
    logger_util.cc
    ${SOURCES}
  )
  target_link_libraries(logger_test PRIVATE GTest::GTest GTest::Main glog
    cyber_proto protobuf
  )

  # 添加测试到 CTest
  add_test(NAME LoggerUtilTest COMMAND logger_util_test)
  add_test(NAME LogFileObjectTest COMMAND log_file_object_test)
  add_test(NAME LoggerTest COMMAND logger_test)
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
#         DESTINATION include/cyber/logger
#         FILES_MATCHING PATTERN "*.h")
