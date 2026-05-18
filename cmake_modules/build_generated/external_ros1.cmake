# cmake_modules/find_ros1.cmake

# 独立构建 dds ros1 ros2
include(ExternalProject)

# 添加独立构建的 ExternalProject
# ros1_ws 它仅在主工程的构建过程中有意义，是一个管理用的标识符。
ExternalProject_Add(ros1_ws
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/ros1           # 指向 ros1 的 ws 目录
  BINARY_DIR ${CMAKE_SOURCE_DIR}/ros1/build     # 构建目录
  CONFIGURE_COMMAND ""                          # 禁用默认 CMake 配置步骤
  BUILD_COMMAND                                 # 使用 catkin build 替代 catkin_make
    catkin build
    -DPYTHON_EXECUTABLE=/usr/bin/python3
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -j8
  INSTALL_COMMAND ""                                # ROS 用不到 install，禁用
  # LOG_BUILD ON
)

# 添加安装路径供主工程使用
include_directories(${CMAKE_SOURCE_DIR}/ros1/devel/include)
# link_directories(${CMAKE_SOURCE_DIR}/ros1/devel/lib)