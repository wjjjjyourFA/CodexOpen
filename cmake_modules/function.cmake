# cmake_modules/function.cmake

function(cyber_add_module NAME)
  file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}/*.cc
  )
  list(FILTER SRCS EXCLUDE REGEX ".*_test\\.cc$")
  add_library(${NAME} STATIC ${SRCS})
  target_include_directories(${NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${NAME})
endfunction()

function(install_module TARGET_NAME MODULE_PATH)
  # TARGET_NAME: add_library / add_executable 的目标
  # MODULE_PATH: 模块路径，如 modules/drivers/camera
  # PREFIX: 可选安装前缀（默认使用 CMAKE_INSTALL_PREFIX）

  # message(STATUS "INSTALL_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
  
  # ===== 检查 target 是否存在 =====
  if(NOT TARGET ${TARGET_NAME})
    message(FATAL_ERROR "Target ${TARGET_NAME} does not exist")
  endif()

  # 第3个及以后参数
  set(EXCLUDE_LIST ${ARGN})

  # ===== 获取 target 类型 =====
  get_target_property(TARGET_TYPE ${TARGET_NAME} TYPE)

  # ===== 计算 MODULE_PATH 层级 =====
  string(REPLACE "/" ";" PATH_LIST ${MODULE_PATH})
  list(LENGTH PATH_LIST DEPTH)

  # 回退层级 = MODULE_PATH 深度 + 1（因为还有 bin）
  math(EXPR BACK_STEPS "${DEPTH} + 1")

  set(RPATH_PREFIX "")
  foreach(i RANGE 1 ${BACK_STEPS})
    set(RPATH_PREFIX "${RPATH_PREFIX}../")
  endforeach()

  # 去掉最后一个 /
  string(REGEX REPLACE "/$" "" RPATH_PREFIX "${RPATH_PREFIX}")

  # ===== 生成最终 RPATH =====
  set(FINAL_RPATH "$ORIGIN/${RPATH_PREFIX}/lib/${MODULE_PATH}")

  # ===== 只对需要的 target 设置 RPATH =====
  if(TARGET_TYPE STREQUAL "EXECUTABLE" OR
     TARGET_TYPE STREQUAL "SHARED_LIBRARY" OR
     TARGET_TYPE STREQUAL "MODULE_LIBRARY")

    set_target_properties(${TARGET_NAME} PROPERTIES
      INSTALL_RPATH "${FINAL_RPATH}"
    )

  endif()

  # 安装库（静态/动态）
  install(TARGETS ${TARGET_NAME}
    EXPORT ${TARGET_NAME}Targets
    ARCHIVE DESTINATION lib/${MODULE_PATH}  # 静态库
    LIBRARY DESTINATION lib/${MODULE_PATH}  # 动态库
    RUNTIME DESTINATION bin/${MODULE_PATH}  # 可执行文件
  )

  # 安装头文件（保持源目录层级），
  # way 1 假设模块源码在 ${CMAKE_CURRENT_SOURCE_DIR} 下
  # set(INSTALL_ARGS
  #   DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  #   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${MODULE_PATH}
  #   FILES_MATCHING
  #     PATTERN "*.h"
  #     PATTERN "*.hpp"
  #     PATTERN "*.inl"

  #   PATTERN "build"      EXCLUDE
  #   PATTERN "Build"      EXCLUDE
  #   PATTERN "Release"    EXCLUDE
  #   PATTERN "Debug"      EXCLUDE
  #   PATTERN ".git"       EXCLUDE
  #   PATTERN "CMakeFiles" EXCLUDE
  #   PATTERN "*.pro"      EXCLUDE
  # )

  # foreach(item ${EXCLUDE_LIST})
  #   list(APPEND INSTALL_ARGS
  #     PATTERN "${item}" EXCLUDE
  #     PATTERN "${item}/*" EXCLUDE
  #   )
  # endforeach()

  # install(${INSTALL_ARGS})

  # 收集头文件
  file(GLOB_RECURSE HEADER_FILES
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "*.h"
    "*.hpp"
    "*.inl"
  )
    
  # 安装头文件
  foreach(H ${HEADER_FILES})
    set(SKIP FALSE)

    # 默认排除目录
    if(H MATCHES "^build/")
      set(SKIP TRUE)
    endif()

    if(H MATCHES "^Build/")
      set(SKIP TRUE)
    endif()

    if(H MATCHES "^Release/")
      set(SKIP TRUE)
    endif()

    if(H MATCHES "^Debug/")
      set(SKIP TRUE)
    endif()

    if(H MATCHES "^CMakeFiles/")
      set(SKIP TRUE)
    endif()

    if(H MATCHES "^devel/")
      set(SKIP TRUE)
    endif()

    if(H MATCHES "^install/")
      set(SKIP TRUE)
    endif()

    if(H MATCHES "^catkin_generated/")
      set(SKIP TRUE)
    endif()

    # 用户额外排除目录
    foreach(item ${EXCLUDE_LIST})
      if(H MATCHES "^${item}/")
        set(SKIP TRUE)
      endif()
    endforeach()

    if(SKIP)
      continue()
    endif()

    # 保持目录结构
    get_filename_component(H_DIR ${H} DIRECTORY)
    install(
      FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${H}
      DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/${MODULE_PATH}/${H_DIR}
    )
  endforeach()
endfunction()