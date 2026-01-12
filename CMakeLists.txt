cmake_minimum_required(VERSION 3.5)

project(CodexOpen VERSION 10.0.0)

# 设置 CMake 构建阶段的并行编译线程数为 12。
set(CMAKE_BUILD_PARALLEL_LEVEL 16)

# 禁用所有编译警告
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# 设置库文件的安装路径
set(THIRD_LIB_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install)
message(STATUS "THIRD_LIB_DIR: ${THIRD_LIB_DIR}")

# 模块路径添加到 CMake 查找模块的路径列表
# 并不会自动执行该路径下的所有 .cmake 文件
# find_package() 或 include() 时，CMake 会查找这个路径下的 .cmake 文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 加载并执行自选模块
include(${CMAKE_SOURCE_DIR}/cmake_modules/base.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/function.cmake)

# system_provided
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/gtest.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/glog.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/gflags.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/eigen3.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/pcl.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/opencv.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/system_provided/protobuf.cmake)

# manual_generated
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/bvar.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/abseil.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/osqp.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/json.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/gperftools.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake_modules/manual_generated/proj.cmake)

include(${CMAKE_SOURCE_DIR}/cmake_modules/other.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/final.cmake)

# ccache
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  message(STATUS "#### use ccache")
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
set(CYBER_MSGS_CACHE ON CACHE BOOL "generate CodexOpen proto msgs flag.")
#### python ####

include_directories(${CMAKE_SOURCE_DIR} ${DEPENCENCY_INCLUDE_DIRS})
link_directories(${DEPENCENCY_LIB_DIRS})

#### proto ####
# 提前优先编译消息的 proto
# add_subdirectory(modules/common_msgs)
file(GLOB_RECURSE CYBER_PROTO_FILES
  "${CMAKE_SOURCE_DIR}/cyber/*.proto"
  # "${CMAKE_SOURCE_DIR}/modules/*.proto"
  "${CMAKE_SOURCE_DIR}/modules/common_msgs/*.proto"
  "${CMAKE_SOURCE_DIR}/modules/drivers/camera/proto/*.proto"
)

list(FILTER CYBER_PROTO_FILES EXCLUDE REGEX ".*/perception/*")
list(FILTER CYBER_PROTO_FILES EXCLUDE REGEX ".*/third-party/.*")

# foreach(file IN LISTS CYBER_PROTO_FILES)
#   message(STATUS "Proto: ${file}")
# endforeach()

# 每次 CMake 配置阶段都会执行一次（即每次运行 cmake 命令都会调用它）
# 但如果只是执行 make 或 ninja 编译构建，不会再重新执行 execute_process()
# 每次修改 CMakeLists.txt 文件后，会重新执行 CMake 配置阶段，
# 但编译阶段会根据文件时间戳判断是否重新生成或编译，所以 proto 文件未修改，不会自动重新生成
if(${CYBER_MSGS_CACHE})
  set(CYBER_MSGS_CACHE OFF CACHE BOOL "generate CodexOpen proto msgs flag." FORCE)

  if(NOT PROTOC MATCHES "NOTFOUND" AND PROTOBUF_FOUND)
    message(=============================================================)
    message("-- CodexOpen -- Protobuf Found, Protobuf Support is turned On!")
    message(=============================================================)
    add_definitions(-DPROTO_FOUND)

    include_directories(${PROTOBUF_INCLUDE_DIRS})
    foreach(FIL ${CYBER_PROTO_FILES})
      # message("#### generate proto file: ${FIL}")
      get_filename_component(FIL_WE ${FIL} NAME_WE)
      # string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
      # string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})
      get_filename_component(FILE_NAME ${FIL} NAME)
      get_filename_component(FILE_PATH ${FIL} PATH)
      execute_process(
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        -I${CMAKE_SOURCE_DIR}
        --cpp_out=${CMAKE_SOURCE_DIR}
        --python_out=${CMAKE_SOURCE_DIR}
        ${FIL}
        # RESULT_VARIABLE ret
        # OUTPUT_VARIABLE out
        # ERROR_VARIABLE err
      )

      # debug
      # if(NOT ret EQUAL 0)
      #   message(FATAL_ERROR "❌ protoc failed on ${FIL}\nError:${err}")
      # else()
      #   message(STATUS "✅ Successfully generated ${FIL}")
      # endif()
    endforeach()
  endif(NOT PROTOC MATCHES "NOTFOUND" AND PROTOBUF_FOUND)
endif()

file(GLOB CYBER_PROTO_SRCS CONFIGURE_DEPENDS
  "cyber/proto/*.pb.cc"
  # "cyber/proto/*.pb.h"
  "cyber/benchmark/*.pb.cc"
  # "cyber/benchmark/*.pb.h"
  "cyber/ros_bridge/proto/*.pb.cc"
  # "cyber/ros_bridge/proto/*.pb.h"
  "cyber/examples/proto/*.pb.cc"
  # "cyber/examples/proto/*.pb.h"
  "modules/common_msgs/**/*.pb.cc"
  # "modules/common_msgs/**/*.pb.h"
  "modules/drivers/camera/proto/*.pb.cc"
  # "modules/drivers/camera/proto/*.pb.h"
)
# foreach(file IN LISTS CYBER_PROTO_SRCS)
#   message(STATUS "Proto Source: ${file}")
# endforeach()
#### proto ####

# 添加子目录
# add_subdirectory(cyber)
# add_subdirectory(docker)
# add_subdirectory(modules)
# add_subdirectory(third_party)
# add_subdirectory(tools)

