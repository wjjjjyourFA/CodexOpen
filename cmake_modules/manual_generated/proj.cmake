# cmake_modules/proj.cmake

# 手动安装
set(PROJ_ROOT ${THIRD_LIB_DIR}/PROJ)

set(PROJ_INCLUDE_DIRS
  ${PROJ_ROOT}/include
)

set(PROJ_LIBRARY_DIRS
  ${PROJ_ROOT}/lib
)

set(PROJ_LIBRARIES
  proj
)

# include_directories(${PROJ_INCLUDE_DIRS})
# link_directories(${PROJ_LIBRARY_DIRS})

message(STATUS "Using PROJ from: ${PROJ_ROOT}")
message(STATUS "PROJ_LIBRARIES: ${PROJ_LIBRARIES}")