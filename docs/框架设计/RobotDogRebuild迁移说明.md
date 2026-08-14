# RobotDog rebuild 流程迁移说明

## 框架内结构

`robot_dog_rebuild` 已接入 CodexOpen 顶层 CMake/Ninja 构建。三个进程分别安装
为独立可执行文件，总控脚本只负责装配、日志和生命周期。

| 模块 | 核心/实现位置 | ROS1 接口位置 |
| --- | --- | --- |
| FAST-LIO | `modules/localization/fast_lio` | `ros1/src/localization/fast_lio` |
| 手动回环会话导出 | `tools/manual_loop_session_exporter` | `ros1/src/tools/manual_loop_session_exporter` |
| 状态监控 | `modules/monitor/robot_status_monitor` 的标准配置 | `ros1/src/monitor/robot_status_monitor` |

公共 YAML 参数加载位于 `ros1/src/common`，不再让定位或工具依赖导航适配包。
Fast-LIO 公式、IMU 处理、IKD-Tree、关键帧判定和导出公式均未修改；迁移新增的
代码只处理 ROS 消息、配置装载、输出目录准备和进程编排。

## 独立安装目标与配置

```text
install/bin/modules/localization/fast_lio/robot_dog_fast_lio_ros1
install/bin/tools/manual_loop_session_exporter/manual_loop_session_exporter_ros1
install/bin/modules/monitor/robot_status_monitor/robot_status_monitor_ros1
```

配置分为算法/运行参数和接口参数：

```text
install/bin/config/RobotDogFastLio/
├── RobotDogFastLio.yaml
├── InterfaceRobotDog.yaml
└── RobotDogFastLio.rviz

install/bin/config/ManualLoopSessionExporter/
├── ManualLoopSessionExporter.yaml
└── Interface.yaml

install/bin/config/RobotStatusMonitor/
├── RobotStatusMonitor.ini
└── Interface.ini
```

旧 YAML 中已有的字段在 runtime 与 interface 合并后名称和值一致；新增字段仅为
原来硬编码的四个输出 topic，使 ROS 接口也能由 `InterfaceRobotDog.yaml` 管理。

## 话题与 TF

输入：

- `/livox/lidar`：Livox `CustomMsg`
- `/imu/data`：`sensor_msgs/Imu`

定位输出：

- `/lio/odom`
- `/state_estimation`
- `/lio/path`
- `/registered_scan`
- 动态 TF：`map -> sensor`

`/lio/path` 的 `Path.header.frame_id` 和每个 `PoseStamped.header.frame_id` 都直接
使用 `map`；TF 仍按定位结果发布 `map -> sensor`，没有为 path 另造坐标系。

导出器同步 `/state_estimation` 与 `/registered_scan`。默认每移动 `1.5 m` 保存
关键帧，输出 PCD、TUM 位姿和 G2O 位姿图。

## 构建与启动

```bash
source /opt/ros/noetic/setup.bash
cd /workspaces/CodexOpen-main/build
cmake -G Ninja .. \
  -DBUILD_ROS1=ON \
  -DBUILD_ROS2=OFF \
  -DBUILD_TESTING=OFF \
  -DCMAKE_INSTALL_PREFIX=/workspaces/CodexOpen-main/install
ninja -j2
ninja install
```

启动 Livox 驱动后运行：

```bash
cd /workspaces/CodexOpen-main
install/scripts/robot_dog_rebuild.sh
```

无桌面环境：

```bash
ROBOT_DOG_RVIZ=false ROBOT_DOG_STATUS_MODE=terminal \
  install/scripts/robot_dog_rebuild.sh
```

可用环境变量覆盖配置和输出位置，例如
`ROBOT_DOG_SESSION_ROOT`、`ROBOT_DOG_FAST_LIO_CONFIG`、
`ROBOT_DOG_FAST_LIO_INTERFACE`。默认仍保持原流程的 GUI 状态监控和 RViz。

迁移流程不再依赖 catkin bringup 包和 launch 文件；框架原生入口是安装后的
`scripts/robot_dog_rebuild.sh`。详细使用见 [WS_LPNC 使用手册](../WS_LPNC框架/使用手册.md)。

## 验证

- 顶层 CMake 配置成功，三个独立目标均已编译、安装；
- 旧 Fast-LIO、导出器、状态监控的已有配置字段与新拆分结果逐项相同；
- `install/scripts/robot_dog_rebuild.sh --check` 通过；
- 无传感器、无 RViz 的 8 秒启动测试中，三个进程均保持运行并由脚本正常回收；
- 日志确认 `/livox/lidar`、`/imu/data`、`/state_estimation`、
  `/registered_scan` 等接口从标准配置正确装载。
