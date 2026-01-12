# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_plugin_manager)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

if(SRCS)
  add_library(${PROJECT_NAME} STATIC ${SRCS})
  target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# 安装头文件（如果你希望其他模块使用这个库）
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/cyber/plugin_manager
#         FILES_MATCHING PATTERN "*.h")