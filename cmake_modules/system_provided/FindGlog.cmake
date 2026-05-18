# cmake_modules/glog.cmake

# find_package(glog REQUIRED)

# include_directories (${GLOG_INCLUDE_DIRS})

# FindGlog.cmake
# ---------------
#  GLOG_FOUND
#  GLOG_INCLUDE_DIRS
#  GLOG_LIBRARIES

include(FindPackageHandleStandardArgs)

set(GLOG_ROOT_DIR "" CACHE PATH "Root directory of Google glog")

# 1. pkg-config (apt glog)
find_package(PkgConfig QUIET)
if (PkgConfig_FOUND)
  pkg_check_modules(PC_GLOG QUIET libglog)
endif()

# 2. include dir
find_path(GLOG_INCLUDE_DIR
  NAMES glog/logging.h
  HINTS
    ${PC_GLOG_INCLUDE_DIRS}
    ${GLOG_ROOT_DIR}
  PATHS
    /usr/include
    /usr/local/include
)

# 3. library
find_library(GLOG_LIBRARY
  NAMES glog
  HINTS
    ${PC_GLOG_LIBRARY_DIRS}
    ${GLOG_ROOT_DIR}
  PATHS
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    /usr/local/lib64
)

# 4. standard handling
find_package_handle_standard_args(Glog
  REQUIRED_VARS GLOG_INCLUDE_DIR GLOG_LIBRARY
)

if (GLOG_FOUND)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
endif()

mark_as_advanced(GLOG_INCLUDE_DIR GLOG_LIBRARY)
