## ros1模块

### colcon
```sh
colcon clean

colcon build  --parallel-workers 8

colcon build  --symlink-install  --parallel-workers 8

colcon build --symlink-install --packages-select my_package
 
colcon build  --symlink-install  --parallel-workers 8 --packages-select my_package
```

## FOR REALSENSE 

```sh
colcon build  --symlink-install  --parallel-workers 8 \
    --cmake-args -DUSE_LIFECYCLE_NODE=OFF
```

```sh
colcon build  --symlink-install  --parallel-workers 8 \
    --cmake-args -DUSE_LIFECYCLE_NODE=OFF \
    --packages-select realsense2_camera_msgs realsense2_camera \
    realsense2_description realsense2_ros_mqtt_bridge
```

## ERROR 1

```
ModuleNotFoundError: No module named 'em’
```

### 解决办法

- 一个广泛应用的方法，但没解决我的问题
```sh
sudo apt-get install python-empy  
```
- 如果上一步没成功，尝试运行这个命令安装，这个解决了我的问题
```sh
pip install empy   
```

[SUCCESS]

`colcon build`
   - 会调用不同的 python 编译器去编译，很大概率会与 conda 的 python 编译器版本不一致
   - 最好，直接在原生环境中使用 ros2
   
```sh
conda deactivate

python --version  # 我这里是3.8.10

pip3 install empy==3.3.2
pip3 install colcon-core

colcon build 
```

## ERROR 2

### 解决办法

把 `paddle`等第三方库拿出去该工程，编译完成再拿回来

## ERROR 3

### 解决办法

```sh
sudo apt install ros-foxy-image-transport-plugins
```

```sh
ros2 topic pub /example_topic std_msgs/msg/String "{data: 'Hello ROS 2'}" --rate 10
```
