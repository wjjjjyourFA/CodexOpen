# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_service_discovery_role)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
      
  # 创建 可执行文件
  add_executable(role_test role_test.cc role.cc)
  target_link_libraries(role_test PRIVATE GTest::GTest GTest::Main glog
    cyber_proto protobuf)

  # 添加测试到 CTest
  add_test(NAME RoleTest COMMAND role_test)
endif()