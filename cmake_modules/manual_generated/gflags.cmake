# cmake_modules/gflags.cmake

# 手动安装，已在系统中安装就不要再启用
set(GFLAGS_ROOT ${THIRD_LIB_DIR}/gflags)

set(GFLAGS_INCLUDE_DIRS
  ${GFLAGS_ROOT}/include
)

set(GFLAGS_LIBRARY_DIRS
  ${GFLAGS_ROOT}/lib
)

set(GFLAGS_LIBRARIES
  gflags
)

# include_directories(${GFLAGS_INCLUDE_DIRS})
# link_directories(${GFLAGS_LIBRARIES})