# Localization_robot 有图定位迁移说明

## 1. 迁移边界

本次迁移对象是：

```text
aa_need_to_trans/TOO/Localization_robot
aa_need_to_trans/TOO/bin/parameters/Localization.yaml
aa_need_to_trans/src/super_lio_robot/map/global_map_manual_opt.pcd
```

需要特别区分：旧 `aa_need_to_trans/script/robot_dog_slam.sh` 启动的是
`super_lio relocation_dog_output.launch`，不是 `TOO/Localization_robot` 可执行文件。
本次按用户指定迁移后者，不把两个定位算法合并，也不改动 `super_lio`。

目标效果是加载已有 PCD 地图，通过 RViz `/initialpose` 提供初值，然后持续输出
有图定位结果。

## 2. 新框架位置

```text
modules/localization/prior_map_localization/
├── CMakeLists.txt
├── include/                         旧算法头文件及 ikd-tree
├── src/
│   ├── ImuProcess.cpp
│   ├── MapProcess.cpp
│   └── PreProcess.cpp
└── config/runtime/
    ├── PriorMapLocalization.yaml    算法参数
    ├── Interface.yaml               topic/frame 接口
    ├── data/global_map_manual_opt.pcd
    └── rviz/PriorMapLocalization.rviz

ros1/src/localization/prior_map_localization/
├── include/                         ROS1 类和旧消息兼容头
├── msg_definitions/                 兼容头对应的原始 msg 定义
└── src/
    ├── localization_ros1.cpp
    └── run_prior_map_localization_ros1.cpp

script/robot_dog_localization.sh     流程编排
```

核心目标和 ROS1 可执行文件分别安装为：

```text
install/lib/modules/localization/prior_map_localization/libprior_map_localization_core.a
install/bin/modules/localization/prior_map_localization/localization_robot_ros1
```

## 3. 参数与资源

算法数值从旧 `Localization.yaml` 原值迁入
`PriorMapLocalization.yaml`。先验地图的旧绝对路径改成模块资源相对路径，运行时由
适配入口传入安装后的绝对路径；这是部署路径调整，不是算法参数调整。

旧源码读取最大迭代次数时使用了不存在的 `mapping/max_iteration`，而配置实际是
`mapprocess/max_iteration: 5`。迁移后读取实际存在的键，继续使用配置中的原值
`5`，没有重新调参。

旧 `RosTopicFile.ini` 中同一个键出现多次，旧解析器以最后一次出现为准。新
`Interface.yaml` 保存其最终有效值，并补充原来硬编码在 C++ 中的接口：

| 类型 | 默认值 |
| --- | --- |
| Livox 点云 / IMU | `/livox/lidar`、`/imu/data` |
| 标准点云 / IMU | `/rslidar_points/main`、`SensorMsgsIMU` |
| RViz 初值 | `/initialpose` |
| 地图 / 注册帧 | `/world_state/cloud_map`、`/registered_scan` |
| 定位输出 | `/lio/odom`、`/state_estimation`、`/lio/path` |
| frame | `map`、`world`、`sensor`、`body` |

原配置虽然在 `lidar_type: 1` 后写了 `RS` 注释，但旧枚举实际定义
`AVIA = 1`。迁移保留代码真实语义，默认仍订阅 Livox 接口，没有擅自根据注释
改成 RS128。

先验 PCD 与 RViz 文件在迁移前后 SHA-256 一致。运行日志不再写入旧源码目录，
而是由适配入口统一写入当次流程日志目录。

## 4. RViz 生命周期

旧定位节点在 `Run_rviz()` 开始时只发布一次先验地图，而且 publisher 不是
latched。因此新流程严格按以下顺序启动：

```text
ROS master -> RViz（optional）-> 等待 -> localization_robot_ros1（required）
```

RViz 使用 `codexopen_start_optional_process` 登记。公共监控循环忽略 optional
进程退出，所以用户关闭 RViz 后定位节点和 ROS master 继续运行；总控终端
`Ctrl-C` 时才统一清理仍存活的进程。

若通过 `ROBOT_DOG_LOCALIZATION_RVIZ=false` 使用外部 RViz，外部 RViz 应在启动
定位节点前打开并订阅 `/world_state/cloud_map`。

## 5. 数据和 TF

```text
/livox/lidar + /imu/data + /initialpose + prior map
                         │
                         v
              prior_map_localization
                         │
       /state_estimation (map -> sensor pose)
       /lio/odom         (world frame)
       /lio/path         (map frame)
       /registered_scan  (map frame)
       TF map -> sensor
```

该节点与 FAST-LIO 都会发布同名定位话题和 `map -> sensor`，两者是可替换的定位
来源，不能在同一 ROS master 中同时作为主定位运行。

按本框架已经确定的定位接口约定，新增 `/lio/path` 的 header 和 pose 均使用
`map`；`/lio/odom` 仍保留旧 `world` frame，动态 TF 仍为 `map -> sensor`。

## 6. 使用

```bash
install/scripts/robot_dog_localization.sh --check
install/scripts/robot_dog_localization.sh
```

RViz 打开并显示地图后，使用 `2D Pose Estimate` 给出平面初值。算法收到第一帧
有效点云和初值后执行原有自动配准，然后开始发布定位结果。

替换地图示例：

```bash
ROBOT_DOG_LOCALIZATION_MAP=/path/to/map.pcd \
  install/scripts/robot_dog_localization.sh
```

## 7. 验证

- `ImuProcess.cpp`、`MapProcess.cpp`、`PreProcess.cpp`、`ikd_Tree.cpp` 与旧工程
  逐字节一致；
- 先验 PCD 与 RViz 配置逐字节一致；
- `localization_robot_ros1` 已通过 Ninja 编译、安装和动态库缺失检查；
- `robot_dog_localization.sh` 已通过 Bash 语法和 `--check` 完整性检查；
- 无传感器短时启动已成功加载 196418 个地图点、建立 ikd-tree 并发布地图；
- RViz 被故意置于无显示环境退出后，定位节点仍能启动并持续运行，验证其为
  非关键进程。

以上验证不替代接入真实 Livox/IMU 后的现场初值和定位精度测试。
