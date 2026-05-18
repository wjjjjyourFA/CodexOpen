# cyber/message/CMakeLists.txt
# cmake_minimum_required(VERSION 3.5)

project(cyber_message LANGUAGES CXX)

set(SOURCES
  # ${CMAKE_SOURCE_DIR}/cyber/binary.cc
  ${CMAKE_SOURCE_DIR}/cyber/message/protobuf_factory.cc
)

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  message("-- cyber message -- Enable Unit Tests is turned On!")

  enable_testing()

  add_executable(message_header_test message_header_test.cc)
  target_link_libraries(message_header_test GTest::GTest GTest::Main glog
    cyber_binary
  )

  add_executable(protobuf_factory_test protobuf_factory_test.cc ${SOURCES})
  target_link_libraries(protobuf_factory_test GTest::GTest GTest::Main glog 
    cyber_binary cyber_proto
  )

  add_executable(raw_message_test raw_message_test.cc ${SOURCES})
  target_link_libraries(raw_message_test GTest::GTest GTest::Main glog 
    cyber_binary cyber_proto
  )

  add_executable(message_traits_test message_traits_test.cc ${SOURCES})
  target_link_libraries(message_traits_test GTest::GTest GTest::Main glog 
    cyber_binary cyber_proto
  )

  # 添加测试到 CTest
  add_test(NAME MessageHeaderTest COMMAND message_header_test)
  add_test(NAME ProtobufFactoryTest COMMAND protobuf_factory_test)
  add_test(NAME RawMessageTest COMMAND raw_message_test)
  add_test(NAME MessageTraitsTest COMMAND message_traits_test)
endif()

# 添加所有模块内源文件（自动收集）
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# 排除测试文件（以 _test.cc 结尾）
list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")

# foreach(file IN LISTS SRCS)
#   message(STATUS "----: ${file}")
# endforeach()

if(SRCS)
  add_library(${PROJECT_NAME} STATIC ${SRCS})
  target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE cyber_binary)
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} 
  INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE cyber_binary)
endif()

install_module(${PROJECT_NAME} cyber/message)