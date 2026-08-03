## ros1模块

20240309 ==> use this 
```sh
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j8
```

20250109 ==> use this 
```sh
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -j8

# for livox 
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -j8 \
  -DROS_EDITION=ROS1
```

`catkin build` ：
- 默认支持并行构建（每个包独立构建）。
- 会根据这些依赖关系的声明，确保先构建依赖包，再构建依赖它的包，因此不会出错。

`catkin_make` ：
- 由于使用全局构建目录，包之间的依赖顺序可能不正确，导致编译失败。
- 并行构建时，可能会在包之间的依赖关系上发生冲突，导致某些头文件或消息文件找不到。