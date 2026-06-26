## 快速入手

使用 CodexOpen 构建的 docker 镜像；
- 镜像只保留基础的库文件，各类编译文件不提供，因此需要下载后，在容器里再次初始化 CodexOpen

docker 镜像的详细介绍：[docker_CodexOpen](./../../docs/安装指南/docker/docker_CodexOpen.md)

### 快速初始化

> 使用 CMakeList，编译 `.proto` 文件，`lib`文件等

0. 将拉取，或下载后加压出的 `CodexOpen-main` 改名 `CodexOpen` ，对齐颗粒度；
1. 在对应的`docker`环境中，修改 `modules/common/environment_conf.h` 文件中的**全局宏**，以适配不同的通信框架：
```c++
#define ENABLE_ROS1
// #define ENABLE_ROS2
// #define ENABLE_DDS
```
2. 编译安装：非测试环境，不建议启用`TESTING`
```sh
cd CodexOpen/build
# 仅编译公用 proto
cmake -G Ninja .. \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DBUILD_TESTING=OFF
# 使用 ros1 环境
cmake -G Ninja .. \
  -DBUILD_ROS1=ON \
  -DBUILD_ROS2=OFF \
  -DBUILD_TESTING=OFF
# 使用 ros2 环境
cmake -G Ninja .. \
  -DBUILD_ROS1=OFF \
  -DBUILD_ROS2=ON \
  -DBUILD_TESTING=OFF
# 多核编译
ninja -j 12
# 编译完成后，必须进行安装，否则 INSTALL 目录下不会出现 可执行文件 和 库文件
ninja install
```
3. 第一次编译完成后，如果需要更换环境进行二次编译。请先删除 `BUILD` 缓存，并检查**全局宏**。
4. `CodexOpen` 代码仍在开发中，仅部分成熟算法支持`CMakeList`一键编译。其余开发代码，请检查`PRO`文件。

### GPU 初始化

本工程提供的镜像，为了缩小镜像体积，均不预安装 `CUDA`、`TensorRT`、`LibTorch`、`PaddlePaddle` 等深度学习网络框架。
请需要使用这些模块的同学，自行在 `MASTER` 主机上安装**适配显卡版本**的 **驱动**、**推理库**（谨慎在容器中安装），详细操作步骤见：[gpu_CodexOpen](./../../docs/安装指南/docker/gpu_CodexOpen.md)

---

> CodexOpen ros1 | ros2

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

为了保存最近的冻结坐标系，当传感器数据传入时，一触发回调，立刻读取当前**位姿**数据队列，选择最近邻时间帧数据，并设置为冻结坐标系。

为了保证位姿数据的**实时性**，那么就不能采用 while() 循环的方式进行触发。

## run

- 无人车实时运行相关的代码，共用一种头文件：
     ==> `modules/common`
- 离线使用的工具类代码，模块内部自定义头文件：
     ==> `tools/common`
- 不同的通信协议中的代码，头文件是互相隔离的。

`Apollo` 使用 `cyber` 框架，通过不断添加不同的 `component` 组件，实现不同模块的加载；
`CodexOpen` 工程只是借用其部分模块的代码结构，不考虑其通信框架，各模块仍按旧有模式拓展；

## exclude

`vscode` 搜索时，排除的文件：
```
*.md, *.pro, *.rst, third_party, third_tools, ros1, ros2, tools, modules/drivers/*, *.txt, *.cmake, Makefile, autoware, cmake_modules, *.bazel, *.log, *.js
```

## 其余文档

- [C++风格说明](./C++风格补充说明.md)