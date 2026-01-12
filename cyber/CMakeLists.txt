cmake_minimum_required(VERSION 3.5)

project(cyber)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# 注意编译顺序
add_subdirectory(base)
add_subdirectory(common)
add_subdirectory(context)
# add_subdirectory(proto)  # 采用顶层编译
add_subdirectory(time)
add_subdirectory(logger)
add_subdirectory(message)
add_subdirectory(data)
add_subdirectory(croutine)
add_subdirectory(scheduler)
add_subdirectory(task)
add_subdirectory(sysmo)
add_subdirectory(timer)
add_subdirectory(class_loader)
add_subdirectory(transport)
add_subdirectory(service_discovery)
add_subdirectory(node)
add_subdirectory(blocker)
add_subdirectory(component)
add_subdirectory(parameter)
add_subdirectory(io)
add_subdirectory(profiler)
add_subdirectory(record)
add_subdirectory(event)
add_subdirectory(statistics)
add_subdirectory(plugin_manager)
add_subdirectory(service)

#### cyber ####
#### 递归匹配所有子目录的 .cc 文件
# file(GLOB_RECURSE CYBER_SRCS "./*.cc")   

# list(FILTER CYBER_SRCS EXCLUDE REGEX ".*\\.pb\\.cc$")
# list(FILTER CYBER_SRCS EXCLUDE REGEX .*test[.]cc)
# list(FILTER CYBER_SRCS EXCLUDE REGEX .*/benchmark/*)
# list(FILTER CYBER_SRCS EXCLUDE REGEX .*/class_loader/test/*)
# list(REMOVE_ITEM CYBER_SRCS "${CMAKE_SOURCE_DIR}/cyber/class_loader/shared_library/sample.cc")

#### 只包含当前目录下的文件
file(GLOB CYBER_SRCS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
# foreach(file IN LISTS CYBER_SRCS)
#   message(STATUS "Cyber: ${file}")
# endforeach()

#### 创建最终的聚合库 cyber
add_library(${PROJECT_NAME}_agg INTERFACE)
target_link_libraries(${PROJECT_NAME}_agg INTERFACE
  cyber_base
  cyber_common
  cyber_context
  # cyber_proto  # 从顶层传入
  cyber_time
  cyber_logger
  cyber_message
  cyber_data
  cyber_croutine
  cyber_scheduler
  cyber_task
  cyber_sysmo
  cyber_timer
  cyber_class_loader
  cyber_transport
  cyber_service_discovery
  cyber_node
  cyber_blocker
  cyber_component
  cyber_parameter
  cyber_io
  cyber_profiler
  cyber_record
  cyber_event
  cyber_statistics
  cyber_plugin_manager
  cyber_service
)

add_library(${PROJECT_NAME} SHARED
  ${CYBER_SRCS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_agg  # 聚合接口库
  cyber_proto          # 单独引入，避免聚合库强依赖 proto
  brpc 
  ${NlohmannJson_LIBRARIES}
  # protobuf 
  ${PROTOBUF_LIBRARIES}
  tinyxml2 uuid atomic 
  ${PROJ_LIBRARIES} 
  # ${GperfTools_LIBRARIES}
  # profiler           # ✅ 提供 HeapProfilerXXX 函数
  fastrtps           # ✅ 提供 eprosima::fastrtps::Time_t
  fastcdr            # ✅ 提供 eprosima::fastcdr::Cdr 等序列化
  dl rt
  gflags glog
  pthread 
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# 安装共享库
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib/cyber         # .so 文件安装路径
  RUNTIME DESTINATION bin/cyber         # 如果有可执行部分也一起装上
  ARCHIVE DESTINATION lib/cyber         # 一般不生成 .a，但写上无碍
)

# cyber .h files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/cyber
  FILES_MATCHING PATTERN "*.h"
)

add_subdirectory(mainboard)
add_subdirectory(tools)

# ==> need proto data && modules/common_msgs
add_subdirectory(python)
add_subdirectory(ros_bridge)  

add_subdirectory(examples)

# apollo conf
file(COPY conf/
  DESTINATION ${CMAKE_BINARY_DIR}/share/conf
)
install(DIRECTORY ${CMAKE_BINARY_DIR}/share/conf/
  DESTINATION share/conf
)

# cyber python api
file(COPY python/cyber_py3
  DESTINATION ${CMAKE_BINARY_DIR}/cyber/python/
  FILES_MATCHING PATTERN "*.py"
)
#### install 1:
# install(DIRECTORY ${CMAKE_BINARY_DIR}/cyber/python/
#   DESTINATION ${CYBER_PYTHON_PATH}
#   FILES_MATCHING
#   PATTERN "*.py"
# )
#### install 2:
file(GLOB_RECURSE CYBER_PY_FILES
  "${CMAKE_BINARY_DIR}/cyber/python/*.py"
)
# 设置源路径前缀
set(PYTHON_SRC_DIR "${CMAKE_BINARY_DIR}/cyber/python")
foreach(pyfile ${CYBER_PY_FILES})
  # 获取相对路径，例如 foo/bar.py
  file(RELATIVE_PATH rel_path "${PYTHON_SRC_DIR}" "${pyfile}")
  # 获取目标路径中相对的目录（dirname）
  get_filename_component(rel_dir "${rel_path}" DIRECTORY)
  # 安装到目标路径+相对目录
  install(FILES "${pyfile}" DESTINATION "${CYBER_PYTHON_PATH}/python/${rel_dir}")
endforeach()

# cyber python protobuf
file(WRITE ${CMAKE_BINARY_DIR}/cyber/proto/__init__.py "import os\n")
file(APPEND ${CMAKE_BINARY_DIR}/cyber/proto/__init__.py "import sys\n")
file(APPEND ${CMAKE_BINARY_DIR}/cyber/proto/__init__.py "sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))\n")
file(COPY proto/
  DESTINATION ${CMAKE_BINARY_DIR}/cyber/proto
  FILES_MATCHING
  PATTERN "*.py"
)
install(DIRECTORY ${CMAKE_BINARY_DIR}/cyber/proto/
  DESTINATION ${CYBER_PYTHON_PROTOBUF_PATH}
)