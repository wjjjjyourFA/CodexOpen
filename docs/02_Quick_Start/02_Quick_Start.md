## 快速入手

使用 CodexOpen 构建的 docker 镜像；
- 镜像只保留基础的库文件，各类编译文件不提供，因此需要下载后，在容器里再次初始化 CodexOpen

docker 镜像的详细介绍：[docker_CodexOpen](notes/docs/安装指南/docker/docker_CodexOpen.md)

### 快速初始化

- CodexOpen protobuf

```sh
cd CodexOpen/build

# 编译公用 proto
cmake -G Ninja .. \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++

# 编译 apollo
ninja -j8
```

- CodexOpen ros1 | ros2

```sh
cd CodexOpen/ros1

catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -j8
```

```sh
cd CodexOpen/ros2

colcon build  --symlink-install  --parallel-workers 8
```

## 单位说明

由 CodexOpen 提供的数据接口：
- 原则上按厘米，0.001弧度，厘米每秒，
  - X 以 float 类型保存 ==>  以 int32_t 类型保存
- 其他单位
  - 经纬度 1e7
- 欧拉角转换关系
  - 传感器驱动层，全部转换到`前左上` `FLU` 坐标系；
  - 感知模块程序，全部修改并默认 `FLU` 坐标系，与 ROS 系统对齐；
  - 控制模块程序，全部修改并默认 `右打为负，逆时针为正`； 

## 数据管线简述

为了保存最近的冻结坐标系，当传感器数据传入时，一触发回调，立刻读取当前 位姿 数据，并设置为冻结。

为了保证 位姿数据 的实时性，那么就不能采用 while() 循环的方式进行触发。

## 其余文档

- [C++风格说明](./C++风格补充说明.md)