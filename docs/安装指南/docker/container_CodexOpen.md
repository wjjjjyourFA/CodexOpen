## 查看 container 库版本

### 查看当前时区

```sh
cat /etc/timezone

ls -l /etc/localtime

date
```

### 查看系统版本

```sh
cat /etc/os-release
```

### 查看 `EIGEN`版本

```sh
grep "EIGEN_WORLD_VERSION" -A 2 /usr/include/eigen3/Eigen/src/Core/util/Macros.h
```

### 查看 `PCL`版本

```sh
grep "PCL_VERSION" /usr/include/pcl-*/pcl/pcl_config.h
```

### 查看 `OPENCV`版本

```sh
pkg-config --modversion opencv4
  
# ubuntu22.04 官方提供的是 libopencv_core.so.4.5d 版本
ls -l /usr/lib/x86_64-linux-gnu/libopencv_*.so*
ls -l /usr/lib/x86_64-linux-gnu/libopencv_photo.so*
    
ls -l /opt/ros/noetic/lib/libopencv_*.so*
ls -l /opt/ros/humble/lib/libopencv_*.so*
```

### 查看 `OPENGL`版本

```sh
apt list --installed | grep mesa

ldconfig -p | grep GL
```

### 查看 `GCC`版本

```sh
gcc --version
```

| GCC 版本      | 默认标准       | 支持的 C++ 标准                   | 备注                                             |
| ------------- | -------------- | --------------------------------- | ------------------------------------------------ |
| **GCC 5–6**   | C++98          | C++11 / C++14                     | 仅基础支持                                       |
| **GCC 7–9**   | C++14          | C++17 可选支持                    | 这阶段是主要兼容 C++14 的版本                    |
| **GCC 10–11** | C++17          | 完全支持 C++17（部分 C++20 特性） | ✅ **GCC 11 是最后一个完全支持 C++14 的主流版本** |
| **GCC 12+**   | C++17 默认启用 | C++20 开始成为主要支持目标        | 新项目通常要求至少 C++17                         |
