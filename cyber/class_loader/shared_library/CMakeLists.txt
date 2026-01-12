# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_class_loader_shared_library)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 创建动态库（生成 libmylib.a）
add_library(cyber_simple SHARED
  sample.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
    
  # 创建可执行文件
  add_executable(shared_library_test 
    shared_library_test.cc
    shared_library.cc
  )
  # 链接动态加载相关的系统库 libdl
  target_link_libraries(shared_library_test GTest::GTest GTest::Main  
    dl cyber_simple
    glog
  )

  # 添加测试到 CTest
  add_test(NAME SharedLibraryTest COMMAND shared_library_test)
endif()

###########################################################################
add_library(cyber_shared_library SHARED
  shared_library.cc
)
target_link_libraries(cyber_shared_library
  dl
)

# 5. 安装动态库
install(TARGETS cyber_simple cyber_shared_library
  LIBRARY DESTINATION lib/cyber/class_loader/shared_library
  ARCHIVE DESTINATION lib/cyber/class_loader/shared_library
  RUNTIME DESTINATION bin/cyber/class_loader/shared_library
)

# 7. 安装头文件，保持目录结构
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#   DESTINATION include/cyber/class_loader/shared_library
#   FILES_MATCHING PATTERN "*.h"
# )

message(STATUS "==== Building shared_library ====")