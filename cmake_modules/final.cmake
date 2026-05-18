# cmake_modules/fianl.cmake

# 部分第三方库文件需要指定编译和安装位置
# 避免整个工程的编译和安装目录混乱
# 重新设置编译和安装目录

# path
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Release 和 Debug 模式的编译标志  不同模式会自动切换
# Debug
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -Wno-return-type -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-return-type -Wno-deprecated-declarations")
# Release
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3")

# debug postfix
set(CMAKE_DEBUG_POSTFIX _d)

# Optional: Add other common settings or helper functions here if needed.
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

# 预先设置依赖库的路径
set(DEPENCENCY_INCLUDE_DIRS
  # ${GLOG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
  # ${NlohmannJson_INCLUDE_DIRS}
  # ${GperfTools_INCLUDE_DIRS}

  ${Python_INCLUDE_DIRS}

  # ${FASTCDR_INCLUDE_DIRS}
  # ${FASTRTPS_INCLUDE_DIRS}
  # ${FASTDDS_INCLUDE_DIRS}
)

set(DEPENCENCY_LIB_DIRS
  ${GLOG_LIBRARY_DIRS}
  ${GFLAGS_LIBRARY_DIRS}
  ${UUID_LIBRARIES_DIRS}
  # ${NlohmannJson_LIBRARY_DIRS}
  # ${GperfTools_LIBRARY_DIRS}

  # ${Python_LIBRARIES_DIRS}

  # ${FASTCDR_LIB_DIRS}
  # ${FASTRTPS_LIB_DIRS}
  # ${FASTDDS_LIB_DIRS}
  # ${FoonathanMemory_LIB_DIRS}
)

set(DEPENCENCY_LIBS
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

# 设置默认装目录，可通过 cmake 命令行覆盖
message(STATUS "==== BEFORE ====")
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT = ${CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT}")
# user maybe set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")
# 只在 “第一次 configure” 有效，后续 configure 不再生效
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  message(STATUS ">>> Overriding default install prefix")
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Install path prefix" FORCE)
endif()
message(STATUS "==== AFTER ====")
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")

# 安装目录下的 lib 目录作为 rpath
# 基于当前可执行文件的位置（$ORIGIN）寻找库文件
# set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
# 固定路径部署，无法打包分发、多版本共存
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# 尽管设置了输出目录和安装目录，
# 但 CMake 不会自动为每个子模块进行安装，
# 除非在每个子模块的 CMakeLists.txt 中明确使用 install() 命令来指定要安装的内容。

# configure_file("shell/setup.bash" "${CMAKE_BINARY_DIR}/shell/setup.bash" @ONLY)
# install(FILES ${PROJECT_SOURCE_DIR}/shell/setup.bash
#   DESTINATION .
# )

# configure_file("shell/env.bash" "${CMAKE_BINARY_DIR}/shell/env.bash" @ONLY)
# install(FILES ${PROJECT_SOURCE_DIR}/shell/env.bash
#   DESTINATION .
# )