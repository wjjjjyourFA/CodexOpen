# cmake_modules/gperftools.cmake

# 手动安装
set(GPERFTOOLS_ROOT ${THIRD_LIB_DIR}/gperftools)

set(GPERFTOOLS_INCLUDE_DIRS
  ${GPERFTOOLS_ROOT}/include
)

set(GPERFTOOLS_LIBRARY_DIRS
  ${GPERFTOOLS_ROOT}/lib
)

# 明确只用一种 tcmalloc（最常见、最安全）
set(GPERFTOOLS_LIBRARIES
  tcmalloc
  # tcmalloc_minimal
)

# 简单直接，全局生效（旧式，但清楚）
# include_directories(${GPERFTOOLS_INCLUDE_DIRS})
# link_directories(${GPERFTOOLS_LIBRARY_DIRS})

message(STATUS "Using gperftools from: ${GPERFTOOLS_ROOT}")
message(STATUS "GPERFTOOLS_LIBRARIES: ${GPERFTOOLS_LIBRARIES}")