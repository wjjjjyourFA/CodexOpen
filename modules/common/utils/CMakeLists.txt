cmake_minimum_required(VERSION 3.16)

project(common_utils)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()

  add_executable(string_util_test
    string_util_test.cc
    string_util.cc
    ${SOURCES}
  )
  target_link_libraries(string_util_test GTest::GTest GTest::Main glog
    absl_base absl_strings
    cyber_binary
  )

  add_test(NAME StringUtilTest COMMAND string_util_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

# 合并指定源文件和自动收集的源文件（避免重复）
list(APPEND SRCS ${SOURCES})
list(REMOVE_DUPLICATES SRCS)

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    absl_base
    absl_strings
    cyber_binary
    nlohmann_json::nlohmann_json
    protobuf::libprotobuf
)

install_module(${PROJECT_NAME} modules/common/utils)
