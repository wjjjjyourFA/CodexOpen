# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_croutine)

# 启用汇编支持（仅需一次）
enable_language(ASM)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 判断平台架构，选择合适的 swap 实现
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(SWAP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/detail/swap_x86_64.S)
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcx16")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(SWAP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/detail/swap_aarch64.S)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加源文件
set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/state.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/detail/routine_context.cc
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
  cyber_scheduler cyber_time atomic
  cyber_node cyber_transport cyber_blocker cyber_logger 
  brpc dl ssl rt crypto 
  fastrtps fastcdr
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
   
  add_executable(croutine_test croutine_test.cc croutine.cc ${SOURCES} ${INIT_SRCS})
  target_link_libraries(croutine_test PRIVATE GTest::GTest GTest::Main 
    ${SWAP_SRC} cyber_proto protobuf 
    cyber_time atomic 
    ${INIT_LIBS}
    brpc dl ssl rt crypto 
    fastrtps fastcdr
    gflags glog uuid
  )

  # 添加测试到 CTest
  add_test(NAME CroutineTest COMMAND croutine_test)
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
#         DESTINATION include/cyber/croutine
#         FILES_MATCHING PATTERN "*.h")