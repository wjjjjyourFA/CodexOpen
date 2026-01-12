# cyber/base/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_base)

# 包含源代码目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

option(BUILD_TESTING "Enable Unit Tests" OFF)

if(BUILD_TESTING)
  message("-- cyber base -- Enable Unit Tests is turned On!")

  enable_testing()

  add_executable(atomic_rw_lock_test atomic_rw_lock_test.cc)
  target_link_libraries(atomic_rw_lock_test GTest::GTest GTest::Main glog)

  add_executable(atomic_hash_map_test atomic_hash_map_test.cc)
  target_link_libraries(atomic_hash_map_test GTest::GTest GTest::Main glog)

  add_executable(bounded_queue_test bounded_queue_test.cc)
  target_link_libraries(bounded_queue_test GTest::GTest GTest::Main glog)

  add_executable(unbounded_queue_test unbounded_queue_test.cc)
  target_link_libraries(unbounded_queue_test GTest::GTest GTest::Main glog)

  add_executable(for_each_test for_each_test.cc)
  target_link_libraries(for_each_test GTest::GTest GTest::Main glog)

  add_executable(object_pool_test object_pool_test.cc)
  target_link_libraries(object_pool_test GTest::GTest GTest::Main glog atomic)

  add_executable(cyber_signal_test cyber_signal_test.cc)
  target_link_libraries(cyber_signal_test GTest::GTest GTest::Main glog)

  # 添加测试到 CTest
  add_test(NAME AtomicRwLockTest COMMAND atomic_rw_lock_test)
  add_test(NAME AtomicHashMapTest COMMAND atomic_hash_map_test)
  add_test(NAME BoundedQueueTest COMMAND bounded_queue_test)
  add_test(NAME UnboundedQueueTest COMMAND unbounded_queue_test)
  add_test(NAME ForEachTest COMMAND for_each_test)
  add_test(NAME ObjectPoolTest COMMAND object_pool_test)
  add_test(NAME CyberSignalTest COMMAND cyber_signal_test)
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
  target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# 安装头文件（如果你希望其他模块使用这个库）
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/cyber/base
#         FILES_MATCHING PATTERN "*.h")