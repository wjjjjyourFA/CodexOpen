# cmake_modules/abseil.cmake

find_package(absl REQUIRED)

# 手动安装；安装时改名：abseil-cpp ==> absl
# set(ABSL_ROOT ${THIRD_LIB_DIR})

# set(ABSL_INCLUDE_DIRS
#   ${ABSL_ROOT}/include/absl
# )

# set(ABSL_LIBRARY_DIRS
#   ${ABSL_ROOT}/lib
# )

# set(ABSL_LIBRARIES
#   absl_strings 
#   absl_base 
#   absl_throw_delegate 
#   absl_str_format_internal
# )

# include_directories(${ABSL_INCLUDE_DIRS})
# link_directories(${ABSL_LIBRARY_DIRS})

