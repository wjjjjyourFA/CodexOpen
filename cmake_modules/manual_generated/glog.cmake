# cmake_modules/glog.cmake

# 手动安装，已在系统中安装就不要再启用
set(GLOG_ROOT ${THIRD_LIB_DIR}/gflags)

set(GLOG_INCLUDE_DIRS
  ${GLOG_ROOT}/include
)

set(GLOG_LIBRARY_DIRS
  ${GLOG_ROOT}/lib
)

set(GLOG_LIBRARIES
  glog
)

# include_directories(${GLOG_INCLUDE_DIRS})
# link_directories(${GLOG_LIBRARIES})