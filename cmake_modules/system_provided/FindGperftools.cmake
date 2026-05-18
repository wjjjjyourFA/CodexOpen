# cmake_modules/gperftools.cmake

# find_package(gperftools REQUIRED)

# include_directories(${GPERFTOOLS_INCLUDE_DIRS})

# FindGperftools.cmake
# ---------------

find_path(GPERFTOOLS_INCLUDE_DIR
  NAMES gperftools/profiler.h
)

find_library(GPERFTOOLS_TCMALLOC
  NAMES tcmalloc
)

find_library(GPERFTOOLS_PROFILER
  NAMES profiler
)

if (NOT GPERFTOOLS_INCLUDE_DIR OR NOT GPERFTOOLS_TCMALLOC)
  message(FATAL_ERROR "gperftools not found")
endif()

set(GPERFTOOLS_LIBRARIES
  ${GPERFTOOLS_TCMALLOC}
  ${GPERFTOOLS_PROFILER}
)

message(STATUS "GPERFTOOLS_INCLUDE_DIR = ${GPERFTOOLS_INCLUDE_DIR}")
message(STATUS "GPERFTOOLS_LIBRARIES   = ${GPERFTOOLS_LIBRARIES}")
