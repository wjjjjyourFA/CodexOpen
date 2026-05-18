# 采用全局构建的方式，弃用；使用分散构建；
message(STATUS "Proto files are generated at CMake configure time")

# 提前优先编译消息的 proto
# add_subdirectory(modules/common_msgs)
file(GLOB_RECURSE PROTO_FILES
  "${CMAKE_SOURCE_DIR}/cyber/*.proto"
  # "${CMAKE_SOURCE_DIR}/modules/*.proto"
  # "${CMAKE_SOURCE_DIR}/modules/common_msgs/*.proto"
  # "${CMAKE_SOURCE_DIR}/modules/drivers/camera/proto/*.proto"
)

# list(FILTER PROTO_FILES EXCLUDE REGEX ".*/perception/*")
# list(FILTER PROTO_FILES EXCLUDE REGEX ".*/third-party/.*")

# foreach(file IN LISTS PROTO_FILES)
#   message(STATUS "Proto: ${file}")
# endforeach()

# 每次 CMake 配置阶段都会执行一次（即每次运行 cmake 命令都会调用它）
# 但如果只是执行 make 或 ninja 编译构建，不会再重新执行 execute_process()
# 每次修改 CMakeLists.txt 文件后，会重新执行 CMake 配置阶段，
# 但编译阶段会根据文件时间戳判断是否重新生成或编译，所以 proto 文件未修改，不会自动重新生成
if(${MSGS_CACHE})
  set(MSGS_CACHE OFF CACHE BOOL "generate CodexOpen proto msgs flag." FORCE)

  if(NOT PROTOC MATCHES "NOTFOUND" AND PROTOBUF_FOUND)
    message(=============================================================)
    message("-- CodexOpen -- Protobuf Found, Protobuf Support is turned On!")
    message(=============================================================)
    add_definitions(-DPROTO_FOUND)

    include_directories(${PROTOBUF_INCLUDE_DIRS})
    foreach(FIL ${PROTO_FILES})
      # message("#### generate proto file: ${FIL}")
      get_filename_component(FIL_WE ${FIL} NAME_WE)
      # string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
      # string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})
      get_filename_component(FILE_NAME ${FIL} NAME)
      get_filename_component(FILE_PATH ${FIL} PATH)
      execute_process(
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        -I${CMAKE_SOURCE_DIR}
        --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
        # --python_out=${CMAKE_CURRENT_BINARY_DIR}
        ${FIL}
        # RESULT_VARIABLE ret
        # OUTPUT_VARIABLE out
        # ERROR_VARIABLE err
      )

      # debug
      # if(NOT ret EQUAL 0)
      #   message(FATAL_ERROR "❌ protoc failed on ${FIL}\nError:${err}")
      # else()
      #   message(STATUS "✅ Successfully generated ${FIL}")
      # endif()
    endforeach()
  endif(NOT PROTOC MATCHES "NOTFOUND" AND PROTOBUF_FOUND)
endif()

file(GLOB PROTO_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_BINARY_DIR}/cyber/proto/*.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/cyber/proto/*.pb.h"
  "${CMAKE_CURRENT_BINARY_DIR}/cyber/benchmark/*.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/cyber/benchmark/*.pb.h"
  "${CMAKE_CURRENT_BINARY_DIR}/cyber/ros_bridge/proto/*.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/cyber/ros_bridge/proto/*.pb.h"
  "${CMAKE_CURRENT_BINARY_DIR}/cyber/examples/proto/*.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/cyber/examples/proto/*.pb.h"
  # "${CMAKE_CURRENT_BINARY_DIR}/modules/common_msgs/**/*.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/modules/common_msgs/**/*.pb.h"
  # "${CMAKE_CURRENT_BINARY_DIR}/modules/drivers/camera/proto/*.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/modules/drivers/camera/proto/*.pb.h"
)
foreach(file IN LISTS PROTO_SRCS)
  message(STATUS "Proto Source: ${file}")
endforeach()

add_library(codex_proto STATIC ${PROTO_SRCS})

target_include_directories(codex_proto
  PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}      # protoc 输出的 .pb.h 文件
    ${CMAKE_SOURCE_DIR}/cyber        # 方便 include "cyber/proto/xxx.pb.h"
)

target_link_libraries(codex_proto
  PUBLIC protobuf::libprotobuf
)