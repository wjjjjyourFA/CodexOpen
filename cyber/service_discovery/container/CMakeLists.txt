# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_service_discovery_container)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/service_discovery/role/role.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
      
  # 创建 可执行文件
  add_executable(graph_test graph_test.cc graph.cc)
  target_link_libraries(graph_test GTest::GTest GTest::Main glog)

  add_executable(single_value_warehouse_test PRIVATE single_value_warehouse_test.cc 
    single_value_warehouse.cc ${SOURCES})
  target_link_libraries(single_value_warehouse_test GTest::GTest GTest::Main glog
    cyber_proto protobuf)

  add_executable(multi_value_warehouse_test PRIVATE multi_value_warehouse_test.cc 
    multi_value_warehouse.cc ${SOURCES})
  target_link_libraries(multi_value_warehouse_test PRIVATE GTest::GTest GTest::Main glog
    cyber_proto protobuf)

  add_executable(warehouse_test single_value_warehouse.cc multi_value_warehouse.cc ${SOURCES})
  target_link_libraries(warehouse_test PRIVATE GTest::GTest GTest::Main glog
    cyber_proto protobuf)

  # 添加测试到 CTest
  add_test(NAME GraphTest COMMAND graph_test)
  add_test(NAME SingleValueWarehouseTest COMMAND single_value_warehouse_test)
  add_test(NAME MultiValueWarehouseTest COMMAND multi_value_warehouse_test)
  add_test(NAME WarehouseTest COMMAND warehouse_test)
endif()