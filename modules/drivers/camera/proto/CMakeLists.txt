# modules/drivers/camera/proto/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(camera_proto)

file(GLOB_RECURSE CAMERA_PROTO_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.proto"
)

codex_add_proto_library(
  NAME camera_proto
  PROTO_ROOT ${CMAKE_SOURCE_DIR}
  PROTO_BASE modules/drivers/camera/proto
  PROTO_FILES ${CAMERA_PROTO_FILES}
  OUT_DIR ${CMAKE_BINARY_DIR}
)

install_module(camera_proto modules/drivers/camera/proto)
