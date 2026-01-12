# cyber/io/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)

project(cyber_proto LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(INSTALL_LIB ${CMAKE_SOURCE_DIR}/install/lib/cyber/libcyber_proto.a)

if(EXISTS ${INSTALL_LIB})
  message(STATUS "Use prebuilt libcyber_proto.a in install dir")
  add_library(cyber_proto STATIC IMPORTED GLOBAL)
  set_target_properties(cyber_proto PROPERTIES
    IMPORTED_LOCATION ${INSTALL_LIB}
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/install/include/cyber/proto)
else()
  # 查找 Protobuf 包
  # find_package(Protobuf REQUIRED 3.6)
  # find_program(PROTOC protoc)
  if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
    message(=============================================================)
    message("-- cyber -- Protobuf Found, Protobuf Support is turned On!")
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
                      --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/../../
                      --python_out=${CMAKE_CURRENT_SOURCE_DIR}/../../
                      ${proto_file})  
    endforeach()
  else(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
    message(=============================================================)
    message("-- cyber -- Protobuf Not Found, Protobuf Support is turned Off!")
    message(=============================================================)
  endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)

  # 自动收集当前目录下所有 .proto 文件
  # file(GLOB PROTO_FILES "*.proto")

  # 收集生成的 .pb.cc 和 .pb.h 文件
  file(GLOB PROTO_SRC CONFIGURE_DEPENDS "${PROTO_FILE_PATH}/*.pb.cc")
  file(GLOB PROTO_HEADER CONFIGURE_DEPENDS "${PROTO_FILE_PATH}/*.pb.h")

  # 将生成的 C++ 文件添加到项目中
  add_library(${PROJECT_NAME} STATIC ${PROTO_SRC} ${PROTO_HEADER})

  # 包含路径
  target_include_directories(${PROJECT_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${PROTO_FILE_PATH}>
      $<INSTALL_INTERFACE:include/cyber/proto>
  )

  # 安装静态库
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib/cyber
    LIBRARY DESTINATION lib/cyber   # 虽然是静态库，也写下，方便统一
    RUNTIME DESTINATION bin/cyber   # 一般静态库没有runtime，但写上无害
  )

  # 安装头文件，保持目录结构
  install(DIRECTORY ${PROTO_FILE_PATH}/
    DESTINATION include/cyber/proto
    FILES_MATCHING PATTERN "*.h"
  )
endif()