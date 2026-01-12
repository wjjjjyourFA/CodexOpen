# tools/common/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(tools_common)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加头文件路径
include_directories(
  /opt/kernelsoftdds/include
  /opt/kernelsoftdds/include/ddscxx
  /opt/kernelsoftdds/include/iceoryx/v2.90.0
)

# 添加库路径
link_directories(
  /opt/kernelsoftdds/lib
)

add_subdirectory(convert)