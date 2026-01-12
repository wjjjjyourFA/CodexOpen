# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_io_bridge)

include(${CMAKE_SOURCE_DIR}/cmake_modules/ros1.cmake)

# 创建I NTERFACE 接口库，不需要源文件，仅用于头文件的引用
# 一种 只存在于 CMake 构建系统中、不会产生任何实际输出文件 的“虚拟库”
add_library(${PROJECT_NAME} INTERFACE)
# 设置接口库的头文件目录
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# only test headers
add_executable(test_io_bridge_headers test_io_bridge_headers.cpp)
target_link_libraries(test_io_bridge_headers PRIVATE cyber_io_bridge)

# 安装目标
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION include/cyber/io_bridge
)

# 安装头文件，保持结构
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/cyber/io_bridge
  FILES_MATCHING PATTERN "*.h"
)