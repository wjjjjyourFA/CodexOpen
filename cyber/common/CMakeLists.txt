# cyber/common/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_common LANGUAGES CXX)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()

  # 创建 environment_test 可执行文件
  add_executable(environment_test 
    environment_test.cc
    ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  )
  target_link_libraries(environment_test GTest::GTest GTest::Main glog)

  # 创建 log_test 可执行文件
  add_executable(log_test 
    log_test.cc
    ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  )
  target_link_libraries(log_test GTest::GTest GTest::Main glog)

  add_executable(macros_test macros_test.cc)
  target_link_libraries(macros_test GTest::GTest GTest::Main glog)

  add_executable(file_test file_test.cc file.cc
    ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  )
  target_link_libraries(file_test PRIVATE GTest::GTest GTest::Main glog protobuf)
  if(TARGET cyber_proto)
    target_link_libraries(file_test PRIVATE cyber_proto)
  endif()

  # 添加测试到 CTest
  add_test(NAME EnvironmentTest COMMAND environment_test)
  add_test(NAME LogTest COMMAND log_test)
  add_test(NAME MacrosTest COMMAND macros_test)
  add_test(NAME FileTest COMMAND file_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

# foreach(file IN LISTS SRCS)
#   message(STATUS "----: ${file}")
# endforeach()

# 创建静态库或动态库（建议静态）
add_library(${PROJECT_NAME} STATIC ${SRCS})

target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    cyber_base
    protobuf::libprotobuf
)
if(TARGET cyber_proto)
  target_link_libraries(${PROJECT_NAME} PUBLIC cyber_proto)
endif()

install_module(${PROJECT_NAME} cyber/common)
