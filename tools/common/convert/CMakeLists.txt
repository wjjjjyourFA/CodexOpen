# tools/common/convert/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(tools_convert)

# 包含头文件目录
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# 创建接口库，不需要源文件
add_library(${PROJECT_NAME} INTERFACE)

# 设置接口库的头文件目录
target_include_directories(${PROJECT_NAME} INTERFACE ${HEADER_DIR})
