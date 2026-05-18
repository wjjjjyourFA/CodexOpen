# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_service_discovery_specific_manager)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES
  ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/global_data.cc
  ${CMAKE_SOURCE_DIR}/cyber/common/file.cc
  ${CMAKE_SOURCE_DIR}/cyber/message/protobuf_factory.cc
  ${CMAKE_SOURCE_DIR}/cyber/transport/common/identity.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/manager.cc
  ${CMAKE_SOURCE_DIR}/cyber/service_discovery/role/role.cc
  ${CMAKE_SOURCE_DIR}/cyber/service_discovery/container/multi_value_warehouse.cc
  ${CMAKE_SOURCE_DIR}/cyber/service_discovery/container/single_value_warehouse.cc
  ${CMAKE_SOURCE_DIR}/cyber/service_discovery/container/graph.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  enable_testing()
      
  # 创建 可执行文件
  add_executable(channel_manager_test channel_manager_test.cc 
    channel_manager.cc ${SOURCES})
  target_link_libraries(channel_manager_test PRIVATE GTest::GTest GTest::Main glog uuid
    cyber_message cyber_proto protobuf 
    cyber_time cyber_transport fastrtps fastcdr)

  add_executable(service_manager_test service_manager_test.cc 
    service_manager.cc ${SOURCES})
  target_link_libraries(service_manager_test PRIVATE GTest::GTest GTest::Main glog uuid
    cyber_message cyber_proto protobuf 
    cyber_time cyber_transport fastrtps fastcdr)

  add_executable(node_manager_test node_manager_test.cc
    node_manager.cc ${SOURCES})
  target_link_libraries(node_manager_test PRIVATE GTest::GTest GTest::Main glog uuid
    cyber_message cyber_proto protobuf 
    cyber_time cyber_transport fastrtps fastcdr)
    
  # 添加测试到 CTest
  add_test(NAME ChannelManagerTest COMMAND channel_manager_test)
  add_test(NAME ServiceManagerTest COMMAND service_manager_test)
  add_test(NAME NodeManagerTest COMMAND node_manager_test)
endif()
