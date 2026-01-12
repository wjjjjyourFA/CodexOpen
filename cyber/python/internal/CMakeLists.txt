cmake_minimum_required(VERSION 3.5)

project(python VERSION 1.0.0)

# Python 扩展模块的 底层 C/C++ 动态库文件名（即 .so 文件）通常以 _ 开头
# _cyber_wrapper.so
# _example.so
# 这是 Python 的惯例，用来区分这个库是底层实现（C/C++编写），而不是纯 Python 的模块。

# Extract major and minor versions
string(REPLACE "." ";" VERSION_LIST "${PYTHON_VERSION}")
list(GET VERSION_LIST 0 PYTHON_VERSION_MAJOR)
list(GET VERSION_LIST 1 PYTHON_VERSION_MINOR)

# Define PY_SSIZE_T_CLEAN if Python version is 3.9 or greater
if(PYTHON_VERSION_MAJOR GREATER_EQUAL 3 AND PYTHON_VERSION_MINOR GREATER_EQUAL 9)
  target_compile_definitions(_cyber_wrapper PRIVATE PY_SSIZE_T_CLEAN)
endif()

add_library(_cyber_wrapper SHARED 
  py_cyber.cc
)
target_link_libraries(_cyber_wrapper
  cyber 
  # ${CYBER_PYTHON_LIB}
  ${Python_LIBRARIES}
)
add_library(py_cyber ALIAS _cyber_wrapper)

add_library(_cyber_record_wrapper SHARED 
  py_record.cc
)
target_link_libraries(_cyber_record_wrapper
  cyber
  # ${CYBER_PYTHON_LIB}
  ${Python_LIBRARIES}
)
add_library(py_record ALIAS _cyber_record_wrapper)

add_library(_cyber_time_wrapper SHARED
  py_time.cc
)
target_link_libraries(_cyber_time_wrapper
  cyber
  # ${CYBER_PYTHON_LIB}
  ${Python_LIBRARIES}
)
add_library(py_time ALIAS _cyber_time_wrapper)

add_library(_cyber_parameter_wrapper SHARED 
  py_parameter.cc
)
target_link_libraries(_cyber_parameter_wrapper
  cyber
  # ${CYBER_PYTHON_LIB}
  ${Python_LIBRARIES}
)
add_library(py_parameter ALIAS _cyber_parameter_wrapper)

message(STATUS "CYBER_PYTHON_INTERNAL_PATH: ${CYBER_PYTHON_INTERNAL_PATH}")

set_target_properties(_cyber_wrapper
  PROPERTIES PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/python/internal"
)
set_target_properties(_cyber_record_wrapper
  PROPERTIES PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/python/internal"
)
set_target_properties(_cyber_time_wrapper 
  PROPERTIES PREFIX "" 
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/python/internal"
)
set_target_properties(_cyber_parameter_wrapper
  PROPERTIES PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cyber/python/internal"
)

install(TARGETS
  _cyber_wrapper 
  _cyber_record_wrapper
  _cyber_time_wrapper
  _cyber_parameter_wrapper
  DESTINATION ${CYBER_PYTHON_INTERNAL_PATH}
)

# 在主目录中生成到指定目录去
# file(WRITE ${CMAKE_BINARY_DIR}/cyber/python/internal/__init__.py "import os\n")
# file(APPEND ${CMAKE_BINARY_DIR}/cyber/python/internal/__init__.py "import sys\n")
# file(APPEND ${CMAKE_BINARY_DIR}/cyber/python/internal/__init__.py "sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))\n")
set(PY_INIT_FILE ${CMAKE_BINARY_DIR}/cyber/python/internal/__init__.py)
file(WRITE ${PY_INIT_FILE} "import os\nimport sys\nsys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))\n")
install(FILES ${PY_INIT_FILE} DESTINATION ${CYBER_PYTHON_INTERNAL_PATH})

