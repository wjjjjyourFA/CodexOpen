cmake_minimum_required(VERSION 3.16)

project(drivers)

add_subdirectory(camera)
add_subdirectory(gnss)
### 采用手动编译
# add_subdirectory(rgbd)
# add_subdirectory(lidar)
