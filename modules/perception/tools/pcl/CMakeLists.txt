# modules/perception/common/camera/params/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(perception_tools_pcl)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

# 合并指定源文件和自动收集的源文件（避免重复）
list(APPEND SRCS ${SOURCES})
list(REMOVE_DUPLICATES SRCS)

add_library(${PROJECT_NAME} STATIC
  ${SRCS}
)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    ${PCL_DEFINITIONS}
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    ${PCL_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
    opencv_core
    opencv_imgproc
    common_math
    ${PCL_LIBRARIES}
)

install_module(${PROJECT_NAME} modules/perception/tools/pcl)
