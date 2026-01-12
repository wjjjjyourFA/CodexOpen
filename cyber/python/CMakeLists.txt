add_subdirectory(internal)

option(BUILD_PYTHON "Build Python binding" ON)

if(BUILD_PYTHON)
  # add_subdirectory(cyber_py3)
endif()