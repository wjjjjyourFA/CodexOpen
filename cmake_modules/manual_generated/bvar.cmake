# cmake_modules/bvar.cmake

set(BVAR_ROOT ${THIRD_LIB_DIR}/brpc)

set(BRPC_INCLUDE_DIRS
  ${BVAR_ROOT}/include
)

set(BRPC_LIBRARY_DIRS
  ${BVAR_ROOT}/lib
)

set(BRPC_LIBRARIES
  brpc
)

include_directories(${BRPC_INCLUDE_DIRS})
# include_directories(
#   ${BRPC_INCLUDE_DIRS}/include/bvar
# )
link_directories(${BRPC_LIBRARY_DIRS})
# file(GLOB BRPC_LIBS "${BRPC_LIBRARY_DIRS}/lib/lib*.a")