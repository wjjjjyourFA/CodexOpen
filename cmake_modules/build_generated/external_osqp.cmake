# cmake_modules/find_osqp.cmake
include(ExternalProject)  # 这行代码必须添加，否则 ExternalProject_Add 无法使用

# find_package(osqp REQUIRED)

# 设置 osqp 的输出目录
set(OSQP_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/install/third_party")
set(OSQP_INSTALL_DIR "${CMAKE_SOURCE_DIR}/install/third_party")

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/install/bin/third_party)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OSQP_INSTALL_DIR})
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OSQP_INSTALL_DIR})

# add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/osqp)

# 检查是否已经编译了 osqp
if (EXISTS "${OSQP_INSTALL_DIR}/lib/libosqp.so")
  message(STATUS "========>> Using precompiled OSQP library")
else()
  message(STATUS "========>> Building OSQP from source")

  # 定义 OSQP 目标
  ExternalProject_Add(
      osqp_project
      PREFIX ${CMAKE_BINARY_DIR}/third_party
      SOURCE_DIR ${CMAKE_SOURCE_DIR}/third_party/osqp
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OSQP_INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
      BUILD_BYPRODUCTS ${OSQP_INSTALL_DIR}/lib/libosqp.so
  )
endif()

# 让 CMake 查找已安装的 OSQP 头文件和库
find_library(OSQP_LIB osqp PATHS ${OSQP_INSTALL_DIR}/lib REQUIRED)
include_directories(${OSQP_INSTALL_DIR}/include)
link_directories(${OSQP_INSTALL_DIR}/lib)

### 二次编译时，最好 CLEAN 一下
