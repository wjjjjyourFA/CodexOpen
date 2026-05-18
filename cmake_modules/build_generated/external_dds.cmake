# cmake_modules/find_dds.cmake

# 独立构建 dds ros1 ros2
include(ExternalProject)

# 添加独立构建的 ExternalProject
# dds_project 它仅在主工程的构建过程中有意义，是一个管理用的标识符。
ExternalProject_Add(dds_project
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/dds/src        # 指向 dds 的 src 子目录
  BINARY_DIR ${CMAKE_SOURCE_DIR}/dds/build      # 构建目录
  CMAKE_ARGS                                    # CMake 配置选项
    -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/dds/install
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}      # 可选：传递构建类型 (Debug, Release 等)
  INSTALL_COMMAND ""
)

# 添加安装路径供主工程使用
include_directories(${CMAKE_SOURCE_DIR}/dds/install/include)
# link_directories(${CMAKE_SOURCE_DIR}/dds/install/lib)