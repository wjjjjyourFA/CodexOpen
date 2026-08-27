cmake_minimum_required(VERSION 3.16)

project(common)

add_subdirectory(utils)
add_subdirectory(math)
add_subdirectory(config)
add_subdirectory(transform)

add_library(modules_common INTERFACE)

target_link_libraries(modules_common
  INTERFACE
    common_utils
    common_math
    common_config
    common_transform
    common_struct
)
