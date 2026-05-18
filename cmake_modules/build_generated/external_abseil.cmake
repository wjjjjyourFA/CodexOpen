# cmake_modules/find_abseil.cmake
include(ExternalProject)  # 这行代码必须添加，否则 ExternalProject_Add 无法使用

# find_package(abseil REQUIRED)

# 设置 abseil 的输出目录
set(ABSEIL_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/install/third_party")
set(ABSEIL_INSTALL_DIR "${CMAKE_SOURCE_DIR}/install/third_party")

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/install/bin/third_party)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ABSEIL_INSTALL_DIR})
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ABSEIL_INSTALL_DIR})

# add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/abseil-cpp)

if (EXISTS "${ABSEIL_INSTALL_DIR}/lib/libabsl_base.so")
  message(STATUS "========>> Using precompiled ABSL library")
else()
  message(STATUS "========>> Building ABSL from source")

  ExternalProject_Add(
      abseil_project
      PREFIX ${CMAKE_BINARY_DIR}/third_party
      SOURCE_DIR ${CMAKE_SOURCE_DIR}/third_party/abseil-cpp
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${ABSEIL_INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
                 -DBUILD_SHARED_LIBS=ON   # 强制编译为共享库
      BUILD_BYPRODUCTS ${ABSEIL_INSTALL_DIR}/lib/libabsl_*.so
  )
endif()

# 让 CMake 查找已安装的头文件和库
find_library(ABSL_LIB absl_base PATHS ${ABSEIL_INSTALL_DIR}/lib REQUIRED)
include_directories(${ABSEIL_INSTALL_DIR}/include)
link_directories(${ABSEIL_INSTALL_DIR}/lib)

# 输出找到的库文件路径
# message(STATUS "========>> ABSL_LIB found: ${ABSL_LIB}")