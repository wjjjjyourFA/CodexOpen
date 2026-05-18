cmake_minimum_required(VERSION 3.16)

project(common)

add_subdirectory(util)
add_subdirectory(math)
add_subdirectory(config)

add_library(modules_common INTERFACE)

target_link_libraries(modules_common
  INTERFACE
    common_util
    common_math
    common_config
)