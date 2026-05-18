# cmake_modules/osqp.cmake

# 手动安装
set(OSQP_ROOT ${THIRD_LIB_DIR}/osqp)

set(OSQP_INCLUDE_DIRS
  ${OSQP_ROOT}/include
)

set(OSQP_LIBRARY_DIRS
  ${OSQP_ROOT}/lib
)

set(OSQP_LIBRARIES
  osqp
)

# include_directories(${OSQP_LIBRARY_DIRS})
# link_directories(${OSQP_LIBRARIES})