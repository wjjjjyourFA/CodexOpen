## 快速初始化

```sh
cd build

# 编译公用 proto
cmake -G Ninja .. \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++

# 编译 apollo
ninja -j8
```

## 单位说明

由 CodexOpen 提供的数据接口：
- 原则上按厘米，0.001弧度，厘米每秒，
  - X 以 float 类型保存 ==>  以 int32_t 类型保存
- 其他单位
  - 经纬度 1e7

## 数据管线简述

为了保存最近的冻结坐标系，当传感器数据传入时，一触发回调，立刻读取当前 位姿 数据，并设置为冻结。

为了保证 位姿数据 的实时性，那么就不能采用 while() 循环的方式进行触发。

## 其余文档

- [C++风格说明](./C++风格补充说明.md)