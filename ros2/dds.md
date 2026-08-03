## 1. 切换 DDS 实现

查看系统安装的 DDS：
```sh
ros2 pkg executables rmw_fastrtps_cpp
ros2 pkg executables rmw_cyclonedds_cpp
```

临时切换：
```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds.xml
```

验证：
```sh
echo $RMW_IMPLEMENTATION
```

## 7. 调试 DDS

查看底层发现：
- Fast DDS：
```sh
export RMW_FASTRTPS_LOG_LEVEL=INFO
```
- CycloneDDS：
```sh
export CYCLONEDDS_LOG_LEVEL=trace
```

然后运行节点，会看到日志：
```
SPDP discovery
SEDP discovery
Participant matched
```