# cmake_modules/base.cmake

# 设置默认构建类型 Release
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# 启用汇编语言（Assembly）的支持
enable_language(ASM)

# disable cmake warning
cmake_policy(SET CMP0043 NEW)
cmake_policy(SET CMP0054 NEW)
# new 使用：<Package>_ROOT 环境变量或变量作为搜索路径。
cmake_policy(SET CMP0074 NEW)
# 设置默认行为，对未显式设置的 policy 生效
set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# 构建共享库
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

message(STATUS "    CMake Info")
message(STATUS "=======================================================")
message(STATUS "    Operation System :  ${CMAKE_SYSTEM}")
message(STATUS "    CPU Architecture : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "    Build Type : ${CMAKE_BUILD_TYPE}${CMAKE_CONFIGURATION_TYPES}")
message(STATUS "    Shared Library  : ${BUILD_SHARED_LIBS}")
message(STATUS "=======================================================")

# 操作系统平台检查
if (WIN32)
  message(STATUS "Platform: Windows")
elseif(APPLE)
  message(STATUS "Platform: Apple")
elseif (UNIX)
  message(STATUS "Platform: Unix")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# camke install
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)

# C++ standard and flags
# 为了适配兼容性，所有代码修改到适配 C++14
# 不再全局设置，而是子模块单独设置
# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 可选：禁止 GNU 扩展，比如 -std=gnu++14 → -std=c++14
set(CMAKE_CXX_EXTENSIONS OFF)  
# set(CMAKE_CXX_COMPILER "/usr/bin/g++")

add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

# openmp
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  message(STATUS "OpenMP found")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()