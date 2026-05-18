# cmake_modules/proj.cmake

find_package(PROJ REQUIRED)

# include_directories(${PROJ_INCLUDE_DIRS})

# PROJ4 是旧名字
# PROJ 是新项目名（从 6.x 开始）
message(STATUS "Found PROJ version in: " ${PROJ_VERSION})
# message(STATUS "PROJ_LIBRARY_DIRS: ${PROJ_LIBRARY_DIRS}")
message(STATUS "PROJ_LIBRARY: ${PROJ_LIBRARIES}")
# message(STATUS "PROJ_INCLUDE_DIRS: ${PROJ_INCLUDE_DIRS}")
