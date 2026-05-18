# cmake_modules/tf2.cmake

# 手动安装
set(TF2_ROOT ${THIRD_LIB_DIR}/tf2)

set(TF2_INCLUDE_DIRS
  ${TF2_ROOT}/include
)

set(TF2_LIBRARY_DIRS
  ${TF2_ROOT}/lib
)

# include_directories(${TF2_INCLUDE_DIRS})
# link_directories(${TF2_LIBRARY_DIRS})
