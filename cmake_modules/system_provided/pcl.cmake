# cmake_modules/pcl.cmake

# PCL 常见依赖的最小集合
# find_package(VTK QUIET)
# set(VTK_FIND_COMPONENTS
#   CommonCore
#   CommonDataModel
#   FiltersCore
# )

# set(VTK_FIND_REQUIRED TRUE)

find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
# include_directories(${PCL_INCLUDE_DIRS})
# PCL 太大，不使用，有需要的在单独的模块中使用
# link_directories(${PCL_LIBRARY_DIRS})
# link_libraries(${PCL_LIBRARIES})