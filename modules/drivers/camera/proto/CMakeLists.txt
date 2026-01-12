# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(proto)

# 查找 Protobuf 包
find_package(Protobuf REQUIRED 3.6)
find_program(PROTOC protoc)
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  message(=============================================================)
  message("-- driver camera -- Protobuf Found, Protobuf Support is turned On!")
  message(=============================================================)
  add_definitions(-DPROTO_FOUND)

  include_directories(${PROTOBUF_INCLUDE_DIRS})
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR})
  SET(PROTO_INCLUDE_PATH ${CMAKE_SOURCE_DIR})
  file(GLOB PROTOBUF_FILELIST ${PROTO_FILE_PATH}/*.proto)
  foreach(proto_file ${PROTOBUF_FILELIST})
    message(STATUS "COMPILING ${proto_file} USING ${PROTOBUF_PROTOC_EXECUTABLE}")
    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                    # --proto_path=${PROTO_FILE_PATH}
                    --proto_path=${PROTO_INCLUDE_PATH}
                    --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/../../../../
                    ${proto_file})  
  endforeach()
else(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  message(=============================================================)
  message("-- driver camera -- Protobuf Not Found, Protobuf Support is turned Off!")
  message(=============================================================)
endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)

# # 自动收集当前目录下所有 .proto 文件
# file(GLOB PROTO_FILES "*.proto")

# 收集生成的 .pb.cc 和 .pb.h 文件
file(GLOB PROTO_SRC "${PROTO_FILE_PATH}/*.pb.cc")
file(GLOB PROTO_HEADER "${PROTO_FILE_PATH}/*.pb.h")

# 将生成的 C++ 文件添加到项目中
# add_library(cyber_proto STATIC ${PROTO_SRC} ${PROTO_HEADER})
