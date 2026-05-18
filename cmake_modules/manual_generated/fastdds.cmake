# cmake_modules/fastdds.cmake

find_package(fastdds QUIET)

set(FASTCDR_ROOT ${THIRD_LIB_DIR}/fast-cdr)

set(FASTCDR_INCLUDE_DIRS
  ${FASTCDR_ROOT}/include
)

set(FASTCDR_LIB_DIRS
  ${FASTCDR_ROOT}/lib
)

# link_directories(${FASTCDR_LIB_DIRS})

set(FASTCDR_LIBRARIES
  fastcdr
)

set(FASTDDS_ROOT ${THIRD_LIB_DIR}/fast-dds)

# 设置 Fast DDS 的 include 路径
set(FASTDDS_INCLUDE_DIRS
  ${FASTDDS_ROOT}/include
)

# 设置 Fast DDS 的 lib 路径
set(FASTDDS_LIB_DIRS
  ${FASTDDS_ROOT}/lib
)

# 设置 Fast DDS 所需的库
set(FASTDDS_LIBRARIES
  fastrtps
)

# include_directories(${FASTCDR_INCLUDE_DIRS} ${FASTDDS_INCLUDE_DIRS})
# link_directories(${FASTDDS_LIB_DIRS})
