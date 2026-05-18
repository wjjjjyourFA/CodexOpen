# cmake_modules/foonathan_memory.cmake

find_package(foonathan_memory REQUIRED)

# 手动安装
set(FoonathanMemory_ROOT ${THIRD_LIB_DIR}/foonathan_memory)

set(FoonathanMemory_INCLUDE_DIRS
  ${FoonathanMemory_ROOT}/include
)

set(FoonathanMemory_LIB_DIRS
  ${FoonathanMemory_ROOT}/lib
)

set(FoonathanMemory_LIBRARIES
  foonathan_memory
)

# include_directories(${FoonathanMemory_INCLUDE_DIRS})
# link_directories(${FoonathanMemory_LIB_DIRS})
