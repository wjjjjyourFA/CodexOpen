# 各个模块的 proto 文件生成，并生成对应的库

function(codex_generate_proto)
  cmake_parse_arguments(ARG ""
    "TARGET_NAME;PROTO_PATH;OUT_DIR"
    "PROTO_FILES"
    ${ARGN}
  )

  if(NOT ARG_PROTO_FILES)
    message(FATAL_ERROR "codex_generate_proto: PROTO_FILES is empty")
  endif()

  set(GENERATED_SRCS "")
  set(GENERATED_HDRS "")

  foreach(PROTO ${ARG_PROTO_FILES})
    # 计算 proto 相对 ARG_PROTO_PATH 的路径（决定输出目录结构）
    file(RELATIVE_PATH REL_PATH ${ARG_PROTO_PATH} ${PROTO})
    # 从相对路径中，取所在目录
    get_filename_component(REL_DIR ${REL_PATH} DIRECTORY)
    # 取文件名（不带后缀）
    get_filename_component(NAME_WE ${PROTO} NAME_WE)
    # message("PROTO: ${PROTO}")
    # message("ARG_PROTO_PATH: ${ARG_PROTO_PATH}")
    # message("REL_PATH: ${REL_PATH}")
    # message("REL_DIR: ${REL_DIR}")

    if(REL_DIR STREQUAL "")
      set(OUT_CC ${ARG_OUT_DIR}/${NAME_WE}.pb.cc)
      set(OUT_H  ${ARG_OUT_DIR}/${NAME_WE}.pb.h)
    else()
      set(OUT_CC ${ARG_OUT_DIR}/${REL_DIR}/${NAME_WE}.pb.cc)
      set(OUT_H  ${ARG_OUT_DIR}/${REL_DIR}/${NAME_WE}.pb.h)
    endif()
    # message("OUT_H: ${OUT_H}")

    list(APPEND GENERATED_SRCS ${OUT_CC})
    list(APPEND GENERATED_HDRS ${OUT_H})

    add_custom_command(
      OUTPUT ${OUT_CC} ${OUT_H}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        -I ${ARG_PROTO_ROOT}
        # --cpp_out=${ARG_OUT_DIR}
        --cpp_out=${CMAKE_BINARY_DIR}
        --python_out=${CMAKE_BINARY_DIR}
        ${PROTO}
      DEPENDS ${PROTO}
      COMMENT "Generating proto: ${REL_PATH}"
      VERBATIM
    )
  endforeach()

  foreach(file IN LISTS GENERATED_SRCS)
    message(STATUS "Proto: ${file}")
  endforeach()

  add_custom_target(${ARG_TARGET_NAME}
    DEPENDS ${GENERATED_SRCS} ${GENERATED_HDRS}
  )

  set(${ARG_TARGET_NAME}_SRCS ${GENERATED_SRCS} PARENT_SCOPE)
  set(${ARG_TARGET_NAME}_HDRS ${GENERATED_HDRS} PARENT_SCOPE)
endfunction()

function(codex_add_proto_library)
  cmake_parse_arguments(ARG ""
    "NAME;PROTO_ROOT;PROTO_BASE;OUT_DIR"
    "PROTO_FILES"
    ${ARGN}
  )

  if(NOT ARG_PROTO_BASE)
    set(PROTO_TARGET_PATH ${ARG_OUT_DIR})
  else()
    set(PROTO_TARGET_PATH ${ARG_OUT_DIR}/${ARG_PROTO_BASE})
  endif()

  codex_generate_proto(
    TARGET_NAME   ${ARG_NAME}_proto_gen
    PROTO_PATH    ${ARG_PROTO_ROOT}
    PROTO_FILES   "${ARG_PROTO_FILES}"
    OUT_DIR       ${ARG_OUT_DIR}
  )

  add_library(${ARG_NAME} STATIC
    ${${ARG_NAME}_proto_gen_SRCS}
  )

  add_dependencies(${ARG_NAME} ${ARG_NAME}_proto_gen)

  target_include_directories(${ARG_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${ARG_OUT_DIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

  target_link_libraries(${ARG_NAME}
    PUBLIC protobuf::libprotobuf
  )

  include(GNUInstallDirs)

  # 安装 proto 生成的头文件（保持目录结构）
  install(
    DIRECTORY ${ARG_OUT_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
      PATTERN "*.pb.h"
  )
endfunction()

