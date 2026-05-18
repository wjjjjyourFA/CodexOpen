cmake_minimum_required(VERSION 3.16)

project(CodexOpen VERSION 1.1.0)

# 设置 CMake 构建阶段的并行编译线程数为 12。
set(CMAKE_BUILD_PARALLEL_LEVEL 16)

# 禁用所有编译警告
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# 设置库文件的安装路径
# set(THIRD_LIB_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install)
set(THIRD_LIB_DIR "/usr/local")
message(STATUS "THIRD_LIB_DIR: ${THIRD_LIB_DIR}")

# 模块路径添加到 CMake 查找模块的路径列表
# 并不会自动执行该路径下的所有 .cmake 文件
# find_package() 或 include() 时，CMake 会查找这个路径下的 .cmake 文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 加载并执行自选模块
include(${CMAKE_SOURCE_DIR}/cmake_modules/base.cmake)

# system_provided
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/gflags.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/FindGlog.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/gtest.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/eigen3.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/pcl.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/opencv.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/protobuf.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/json.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/FindGperftools.cmake)

# manual_generated
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/bvar.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/abseil.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/osqp.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/proj.cmake)

include(${CMAKE_SOURCE_DIR}/cmake_modules/other.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/final.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/function.cmake)

if(BUILD_ROS1)
  include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/ros1.cmake)
endif()

if(BUILD_ROS2)
  include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/ros2.cmake)
endif()

# ccache
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  message(STATUS "#### use ccache ####")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

#### python ####
set(PYTHON_VERSION "3.8")
#### ubuntu 18.04 ####
# include_directories(/usr/include/python3.6)
#### ubuntu 20.04 ####
# include_directories(/usr/include/python3.8)
#### ubuntu 22.04 ####
# include_directories(/usr/include/python3.10)
include_directories(/usr/include/python${PYTHON_VERSION})
# set(PYTHON_VERSION "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
set(CYBER_PYTHON_PATH "lib/python${PYTHON_VERSION}/site-packages/cyber")
set(CYBER_PYTHON_INTERNAL_PATH "bin/cyber/python/internal")
set(CYBER_PYTHON_PROTOBUF_PATH "${CYBER_PYTHON_PATH}/proto")
set(MSGS_CACHE ON CACHE BOOL "generate CodexOpen proto msgs flag.")
#### python ####

include_directories(${CMAKE_SOURCE_DIR} ${DEPENCENCY_INCLUDE_DIRS})
link_directories(${DEPENCENCY_LIB_DIRS})

#### proto ####
# include(${CMAKE_SOURCE_DIR}/cmake_modules/cyber_proto.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/codex_proto.cmake)
#### proto ####

# 添加子目录
add_subdirectory(cyber)
# add_subdirectory(docker)
add_subdirectory(modules)
# add_subdirectory(third_party)
add_subdirectory(tools)