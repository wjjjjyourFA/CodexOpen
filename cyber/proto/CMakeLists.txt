# cyber/proto/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_proto)

file(GLOB_RECURSE CYBER_PROTO_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.proto"
)

codex_add_proto_library(
  NAME cyber_proto
  PROTO_ROOT ${CMAKE_SOURCE_DIR}
  PROTO_BASE cyber/proto
  PROTO_FILES ${CYBER_PROTO_FILES}
  OUT_DIR ${CMAKE_BINARY_DIR}
)

install_module(cyber_proto cyber/proto)
