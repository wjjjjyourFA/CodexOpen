# cyber/context/CMakeLists.txt
# cmake_minimum_required(VERSION 3.5)

project(cyber_context LANGUAGES CXX)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  message("-- cyber context -- Enable Unit Tests is turned On!")

  enable_testing()

  add_executable(context_test context_test.cc)
  target_link_libraries(context_test GTest::GTest GTest::Main glog)

  # 添加测试到 CTest
  add_test(NAME ContextTest COMMAND context_test)
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

if(SRCS)
  add_library(${PROJECT_NAME} STATIC ${SRCS})
  target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} 
  INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
endif()

install_module(${PROJECT_NAME} cyber/context)