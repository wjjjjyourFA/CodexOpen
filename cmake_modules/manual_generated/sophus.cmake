# cmake_modules/sophus.cmake

find_package(Sophus REQUIRED )

# 手动安装
set(Sophus_ROOT ${THIRD_LIB_DIR}/sophus)

set(Sophus_INCLUDE_DIRS
  ${Sophus_ROOT}/include
)

set(Sophus_LIBRARY_DIRS
  ${Sophus_ROOT}/lib
)

# include_directories(${Sophus_INCLUDE_DIRS})
# link_directories(${Sophus_LIBRARY_DIRS})