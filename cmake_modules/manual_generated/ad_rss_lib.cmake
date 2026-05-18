# cmake_modules/ad_rss_lib.cmake

# 手动安装
set(AD_RSS_ROOT "${THIRD_LIB_DIR}/ad_rss_lib")

set(AD_RSS_INCLUDE_DIRS
  ${AD_RSS_ROOT}/include
)

set(AD_RSS_LIBRARY_DIRS
  ${AD_RSS_ROOT}/lib
)

set(AD_RSS_LIBRARIES
  ad-rss
)

# include_directories(${AD_RSS_INCLUDE_DIRS})
# link_directories(${AD_RSS_LIBRARY_DIRS})

message(STATUS "Using ad-rss-lib from: ${AD_RSS_ROOT}")
