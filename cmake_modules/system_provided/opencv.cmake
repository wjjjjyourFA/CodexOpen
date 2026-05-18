# cmake_modules/opencv.cmake

# 优先使用 /usr/local（自己编译的版本）
set(OpenCV_DIR /usr/local/share/opencv4)

find_package(OpenCV REQUIRED)

# include_directories(${OpenCV_INCLUDE_DIRS})
# link_libraries(${OpenCV_LIBS})

# message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
# message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "Opencv_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")