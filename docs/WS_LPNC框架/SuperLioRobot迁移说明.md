# SuperLioRobot 迁移说明

## 1. 迁移结论

原始 `aa_need_to_trans/super_lio_robot` 已迁入 WS_LPNC 顶层构建体系，核心算法位于
`modules/mapping/super_lio_robot`，ROS1 适配代码位于
`ros1/src/mapping/super_lio_robot`。迁移后不再使用独立 catkin 包、独立消息生成和
`roslaunch` 入口，而是复用仓库顶层 CMake、公共 YAML 参数加载器、统一安装函数及
`script/runtime_common.sh`。

模块保留以下三种运行模式：

| 模式 | `--mode` | 算法配置 | 接口配置 | 执行路径 |
|---|---|---|---|---|
| Livox Mid-360 输入 | `livox-mid360` | `SuperLioRobotLivoxMid360.yaml` | `InterfaceLivoxMid360.yaml` | 建图 |
| 四足机器人普通 PCD/兼容输出 | `quadruped-pcd` | `SuperLioRobotQuadrupedPcd.yaml` | `InterfaceQuadrupedPcd.yaml` | 建图 |
| 四足机器人重定位输出 | `quadruped-relocation` | `SuperLioRobotQuadrupedRelocation.yaml` | `InterfaceQuadrupedRelocation.yaml` | 先验图重定位 |

启动脚本还接受原配置名 `livox_360`、`pcd_normal_dog_output`、
`relocation_dog_output` 作为模式别名。

## 2. 迁移前后的目录映射

迁移前的主要目录如下：

```text
aa_need_to_trans/super_lio_robot/
├── CMakeLists.txt
├── package.xml
├── include/{lio,OctVoxMap,common,ros}/
├── src/{lio,ros,apps}/
├── config/
├── launch/
├── msg/
├── rviz/
└── map/
```

迁移后的目录如下：

```text
modules/mapping/
├── CMakeLists.txt
└── super_lio_robot/
    ├── CMakeLists.txt
    ├── LICENSE
    ├── include/{basic,common,lio,OctVoxMap}/
    ├── src/
    └── config/runtime/
        ├── SuperLioRobot*.yaml
        ├── Interface*.yaml
        ├── data/global_map_manual_opt.pcd
        └── rviz/*.rviz

ros1/src/mapping/super_lio_robot/
├── include/super_lio_robot_ros1/ros1_convert.h
└── src/{ros1_convert.cpp,super_lio_robot_app.cpp}

script/super_lio_robot.sh
```

具体映射：

| 原位置 | 新位置或处理方式 |
|---|---|
| `include/lio`、`src/lio` | 核心层 `modules/mapping/super_lio_robot/include/lio` 与 `src` |
| `include/OctVoxMap`、`include/common` | 核心层同名目录 |
| 缺失的 `basic` 数学依赖 | 从 Super-LIO 官方 ROS1 源补齐到核心层 `include/basic` 和 `src/Manifold.cpp` |
| `include/ros`、`src/ros` | 改造成 `ros1/src/mapping/super_lio_robot` 下的 ROS1 适配器 |
| `src/apps` | 合并为统一入口 `super_lio_robot_app.cpp`，由运行模式选择核心类 |
| 三个原 YAML | 拆分成三组算法 YAML 和 Interface YAML |
| `rviz/lio.rviz`、`relocation.rviz` | `config/runtime/rviz` |
| `map/global_map_manual_opt.pcd` | `config/runtime/data` |
| 原 `launch/*.launch` | 由安装后的 `scripts/super_lio_robot.sh --mode ...` 替代 |
| 未被三种模式调用的 `CloudPose*.msg` | 不再生成，避免引入模块私有消息包 |

模块源声明为 GPLv3，因此在新模块目录保留了 GPLv3 `LICENSE`，并安装到
`share/licenses/super_lio_robot`。

## 3. 核心与 ROS1 接口拆分

`SuperLIO` 不再包含任何 ROS 头文件。核心通过
`include/lio/data_interface.h` 中的纯 C++ 抽象接口完成以下交互：

- 同步 LiDAR 与 IMU 测量；
- 注入 ESKF 实例以支持 IMU 高频预测；
- 发布里程计、世界点云和四足机器人兼容点云；
- 发布重定位全局地图并接收初始位姿。

ROS1 适配器负责：

- Livox `CustomMsg` 和标准 `PointCloud2` 转换；
- IMU 坐标与单位转换；
- ROS topic、frame、queue、频率、TF 和 latched publisher；
- 使用 `codexopen_ros1_yaml_param_loader` 装载算法和接口参数；
- 把运行时地图根目录解析为显式路径，不再依赖源码目录宏。

原代码在 `b_rawimu=true` 时未给角速度临时变量赋值。迁移后按参数原注释恢复预期
语义：`true` 表示输入已经是 rad/s，直接使用；`false` 表示输入是 deg/s，转换为
rad/s。

## 4. 参数与资源处理

三份算法配置保留了原配置的参数和值。为符合框架的接口分离约定，以下两项从
`lio/ros` 移入各模式 Interface YAML：

| 原参数 | 新参数 |
|---|---|
| `lio/ros/lidar_topic` | `super_lio_robot/topics/lidar` |
| `lio/ros/imu_topic` | `super_lio_robot/topics/imu` |

原先在 C++ 中硬编码的输出 topic、frame、queue、500 Hz 处理频率、路径采样距离和
全局地图 latch 行为也移入 Interface YAML，但默认值保持不变。

重定位地图按仓库资源约定安装到 `data/`，因此重定位 YAML 中
`map/save_map_dir` 从源码树相对目录 `map` 调整为安装配置树相对目录 `data`；参数
仍表示地图所在目录。其余算法叶子参数与原三份配置逐项等价。原 relocation 配置中
已有但算法未消费的 `hash_map/insert_resolution` 和 `occupy_map/*` 现在会被适配器
读取并保留；核心行为仍与原实现一致。

先验地图迁移前后 SHA-256 均为：

```text
52491f4bc7daaf04d6313dd0b2f6d54ab09cac4d57c6ee30b87f03cc53c0ee5a
```

## 5. 依赖适配

核心目标 `super_lio_robot_core` 依赖：

- Eigen3；
- PCL（common、filters、io、kdtree、registration）；
- oneTBB；
- glog、gflags。

只有 `BUILD_ROS1=ON` 时才构建 `super_lio_robot_ros1`，其额外依赖为：

- roscpp；
- sensor_msgs、nav_msgs、geometry_msgs；
- pcl_conversions；
- tf；
- Livox ROS1 `CustomMsg`（优先使用系统头，也支持仓库
  `tools/data_processor/ros1_message` 中的兼容头）；
- 仓库公共 `codexopen_ros1_yaml_param_loader`。

原 catkin 包中的 `rospy`、`visualization_msgs`、Python、私有 `CloudPose*` 消息生成
均不属于这三种运行路径，迁移后不再作为依赖。

## 6. 构建、安装与运行

使用仓库原有顶层构建命令：

```bash
source /opt/ros/noetic/setup.bash
cmake -S . -B build -G Ninja \
  -DBUILD_ROS1=ON \
  -DBUILD_ROS2=OFF \
  -DBUILD_TESTING=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j2
cmake --install build
```

纯核心隔离构建：

```bash
cmake -S . -B build-core -G Ninja \
  -DBUILD_ROS1=OFF \
  -DBUILD_ROS2=OFF \
  -DBUILD_TESTING=OFF
cmake --build build-core -j2
```

安装后可先检查三种模式的可执行文件和资源：

```bash
install/scripts/super_lio_robot.sh --mode livox-mid360 --check
install/scripts/super_lio_robot.sh --mode quadruped-pcd --check
install/scripts/super_lio_robot.sh --mode quadruped-relocation --check
```

去掉 `--check` 即可启动对应模式。可通过
`SUPER_LIO_ROBOT_RVIZ=false` 关闭 RViz，通过 `SUPER_LIO_ROBOT_OUTPUT_ROOT` 修改建图
输出根目录，通过 `SUPER_LIO_ROBOT_CONFIG_DIR` 使用另一套安装格式兼容的配置。

## 7. 本次验证记录

验证日期：2026-08-25。

- 顶层 `BUILD_ROS1=OFF`：109/109 目标构建成功；
- 顶层 `BUILD_ROS1=ON`：149/149 目标构建成功；
- `super_lio_robot_core` 静态库和 `super_lio_robot_ros1` 可执行文件链接成功；
- 顶层安装成功，核心库、ROS1 可执行文件、三组配置、两份 RViz 配置、先验地图、
  GPLv3 许可证和启动脚本均进入安装树；
- 六份 YAML 均通过解析检查；
- 三种模式均通过安装后 `--check`；
- 三种模式均在本机 ROS master 下完成短时启动，算法 YAML 与 Interface YAML 装载及
  核心初始化成功；重定位模式实际读取 196418 个先验地图点并建立 60186 个体素；
- 核心目录静态扫描未发现 ROS API 或 ROS 消息头；
- 安装后的 ROS1 可执行文件动态依赖未发现 `not found`。

## 8. 仍需实机确认

构建环境没有连接 Mid-360、四足机器人或对应 rosbag，因此还需在目标设备确认：

1. LiDAR 与 IMU topic 的实际消息类型、时间同步和 IMU 角速度单位；
2. 三套外参、盲区盒和量程是否与当前机器人安装状态一致；
3. `ax7_compat_output` 对 `/state_estimation`、`/registered_scan` 的下游兼容性；
4. 重定位先验图与现场坐标系匹配，并通过 RViz `/initialpose` 或 YAML 初值完成收敛；
5. `hash_capacity=100000000` 在目标机上的峰值内存占用；
6. 分发集成产物时按 GPLv3 要求处理该模块及其组合产物的许可证义务。
