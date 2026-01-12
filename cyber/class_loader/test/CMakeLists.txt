# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_class_loader_test)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

###########################################################################
# 创建静态库（生成 libmylib.a）
add_library(cyber_plugin1 SHARED
  plugin1.cc
)
target_link_libraries(cyber_plugin1
  PUBLIC
    cyber_class_loader
)

add_library(cyber_plugin2 SHARED
  plugin2.cc 
)
target_link_libraries(cyber_plugin2
  PUBLIC
    cyber_class_loader
)

# 5. 安装动态库
install(TARGETS cyber_plugin1 cyber_plugin2
  LIBRARY DESTINATION lib/cyber/class_loader/test
  ARCHIVE DESTINATION lib/cyber/class_loader/test
  RUNTIME DESTINATION bin/cyber/class_loader/test
)

# 7. 安装头文件，保持目录结构
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#   DESTINATION include/cyber/class_loader/test
#   FILES_MATCHING PATTERN "*.h"
# )