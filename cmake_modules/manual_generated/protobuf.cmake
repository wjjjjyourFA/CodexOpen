# cmake_modules/protobuf.cmake

# 手动安装
set(PROTOBUF_ROOT "${THIRD_LIB_DIR}/protobuf")

set(PROTOBUF_INCLUDE_DIRS
  ${PROTOBUF_ROOT}/include
)

set(PROTOBUF_LIBRARIES
  protobuf
  protoc
)

# include_directories(${PROTOBUF_INCLUDE_DIRS})
# link_directories(${PROTOBUF_LIBRARIES})

set(CMAKE_PREFIX_PATH "${PROTOBUF_ROOT}")

find_package(Protobuf REQUIRED 3.14)

find_program(PROTOC_EXECUTABLE protoc PATHS "${PROTOBUF_ROOT}/bin" NO_DEFAULT_PATH)

# 打印版本 & 诊断
if (Protobuf_FOUND AND NOT PROTOC_EXECUTABLE STREQUAL "PROTOC_EXECUTABLE-NOTFOUND")
  message("=============================================================")
  message("-- CodexOpen -- Protobuf Found at ${PROTOBUF_ROOT}")
  message("-- CodexOpen -- Protobuf Version: ${Protobuf_VERSION}")
  message("-- CodexOpen -- Protoc Executable: ${PROTOC_EXECUTABLE}")
  message("=============================================================")
  add_definitions(-DPROTO_FOUND)
else()
  message(FATAL_ERROR "Protobuf NOT FOUND or protoc NOT FOUND in ${PROTOBUF_ROOT}")
endif()