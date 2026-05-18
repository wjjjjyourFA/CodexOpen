# Look for csparse; note the difference in the directory specifications!
FIND_PATH(CSPARSE_INCLUDE_DIR NAMES cs.h
  PATHS
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
)

FIND_LIBRARY(CSPARSE_LIBRARIES NAMES cxsparse
  PATHS
  /usr/local/lib
  /usr/local/lib64
  /usr/lib
  /usr/lib64
  /opt/local/lib
  /sw/local/lib
  /sw/lib
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  CSPARSE DEFAULT_MSG
  CSPARSE_INCLUDE_DIR 
  CSPARSE_LIBRARIES
)
