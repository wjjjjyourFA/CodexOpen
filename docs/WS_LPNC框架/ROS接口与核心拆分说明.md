# ROS 接口与核心拆分说明

## 1. 更新范围

本轮更新覆盖 `script/` 默认启动链中的全部算法和工具模块：

- `robot_dog_rebuild.sh`：FAST-LIO、手动回环会话导出、状态监控；
- `robot_dog_localization.sh`：先验地图定位；
- `robot_plan_expv2.sh`：ROG-Map、地形分析、世界规划、本地规划、地形航点探索；
- `robot_plan_path.sh`：ROG-Map、地形分析、世界规划、本地规划、航点发布器。

不在启动链中的历史实验程序没有被顺带改写。本次拆分沿用 GroundRemove 的
“核心类 + ROS convert”边界，并进一步把 ROS1 convert 统一放入 `ros1/src`，
使 `modules/` 和 `tools/` 能在 `BUILD_ROS1=OFF` 时独立构建。

## 2. 统一拆分形式

每个 C++ 模块由两个独立目标组成：

```text
lib<module>_core.a
  └── 只包含算法、状态和纯数据接口

<module>_ros1
  ├── run_<module>_ros1.cpp       进程入口和配置文件装载
  ├── ros1_convert.cpp            ROS 消息与核心数据互转
  └── 链接 lib<module>_core.a
```

Python 状态监控采用相同边界：`robot_status_monitor_core.py` 不导入 `rospy`，
`robot_status_monitor_ros1` 负责订阅、时间注入和终端/RViz 展示。

核心层允许使用 PCL、Eigen 等算法数据结构，但不得创建通信节点、读 ROS 参数、
发布 TF、调用 ROS 时钟或使用 ROS 日志宏。所有 topic、service、frame、队列、
latch、消息换算和适配循环频率均由适配层读取 `Interface.yaml`。

## 3. 公共消息边界

规划链使用 `modules/common_struct` 中的纯结构，转换关系如下：

| ROS1 类型 | 核心类型 | 约定 |
| --- | --- | --- |
| `std_msgs/Header` | `common_struct::Header` | ROS 秒/纳秒转换为 `uint64_t` 纳秒 |
| `geometry_msgs/Pose` | `common_struct::Pose` | position 与 quaternion 逐字段转换 |
| `geometry_msgs/PoseStamped` | `common_struct::PoseStamped` | 同时保留消息时间和接收时间 |
| `nav_msgs/Odometry` | `common_struct::Odometry` 或 Pose/速度 | 协方差按 36 个元素原序保存 |
| `nav_msgs/Path` | `common_struct::Path` | header 与每个 pose 的 frame 明确设置 |
| `nav_msgs/OccupancyGrid` | `common_struct::OccupancyGrid` | 栅格数据保持 `int8_t` 语义 |
| `geometry_msgs/Twist` | `common_struct::Twist` | 线速度、角速度逐字段转换 |
| `geometry_msgs/PolygonStamped` | `common_struct::PolygonStamped` | 边界点转为纯 `Vector3f` 数组 |
| `sensor_msgs/PointCloud2` | PCL 点云 | 只在 ROS1 convert 中调用 PCL ROS 转换 |

定位核心使用高频计算所需的 Eigen/PCL 类型和 `double` 秒时间。Livox 数据先转为
`FastLioLivoxPoint` 或纯 `LivoxPointCloud`；标准点云使用 `rs_lidar::Point`、
`velodyne_lidar::Point`，名称不再带有伪 ROS 命名空间。

## 4. 定位模块

### 4.1 FAST-LIO

核心入口为 `FastLioPipeline`：

- `PushLivoxCloud(timestamp, cloud)` 接收纯 Livox 点数组；
- `PushImu(imu)` 接收不含 ROS 头的 IMU 数据；
- `Step(include_registered_scan, dense_registered_scan)` 返回位姿、速度、
  协方差和可选注册点云；
- 输出目录在构造时显式传入，核心递归创建 `online-O/sensor_data/lidar_pcd`
  并验证失败原因，不读取 ROS 参数。

ROS1 适配器负责 Livox/IMU 回调、消息换算、处理循环、路径抽样、Odometry、Path、
PointCloud2 和 `map -> sensor` TF。接口文件已统一为 `Interface.yaml`，其中包含：

- 输入输出 topic；
- `map`、`world`、`sensor` frame；
- lidar、IMU 和输出队列；
- 处理频率与路径最小采样距离；
- 是否发布路径、点云及点云稠密策略。

算法协方差、外参、滤波尺度、机身滤除范围和最大量程仍位于
`RobotDogFastLio.yaml`。

### 4.2 先验地图定位

核心入口为 `PriorMapLocalization`。构造函数显式接收算法配置、PCD 路径和调试
日志目录；核心不读取环境变量，也不包含 Livox ROS、自定义 `self_state` 消息或
ROS 日志。对外提供 Livox、Robosense、Velodyne、IMU 和外部初值的纯输入方法，
`Step()` 返回定位状态、位姿、速度、协方差和注册点云。

ROS1 兼容消息头和 `.msg` 定义仅保留在
`ros1/src/localization/prior_map_localization`。适配器负责：

- 根据 `lidar_type` 选择 Livox 或标准点云/IMU订阅；
- 将 `/initialpose`、GNSS/全局位姿转换为 `ExternalPose`；
- 发布先验地图、轨迹、主定位 Odometry、注册点云和兼容位姿；
- 发布动态 `map -> sensor` TF；
- 按 Interface 中的换算参数处理标准 IMU 轴向和比例。

`/world_state/cloud_map` 使用可配置的 latched publisher，默认开启。路径抽样距离、
所有队列和处理频率均在 `Interface.yaml`；初始化方法、匹配半径和滤波参数在
`PriorMapLocalization.yaml`。

FAST-LIO 与先验定位都提供 `/state_estimation`、`/registered_scan`、`/lio/odom`
和 `/lio/path`，下游无需感知定位实现的差异，但同一 ROS master 中只能选择一个
主定位源。

## 5. 感知模块

### 5.1 ROG-Map

核心入口为 `rog_map::ROGMap`，负责概率地图、滑动地图、膨胀、射线更新、ESDF 和
可视化数据生成。适配器把 Odometry 转为 `rog_map::Pose`、把 PointCloud2 转为
PCL 点云，并负责更新/可视化 timer、publisher、可选动态配置及 TF。

本轮参数边界调整包括：

- `safe_margin` 成为显式初始化并实际参与核心配置的参数；
- 删除没有有效读取/执行路径的 `block_inf_pt`、`kd_tree_en`、
  `blind_filter_en`、`blind_filter_dis`；
- 输入输出队列、更新频率、可视化频率、topic、frame 和点云/里程计换算放入
  `Interface.yaml`；
- 动态配置类型命名为 `rog_map_ros1::VizConfig`，不再携带历史流程名；
- 默认 `broadcast_tf: false`，由定位模块唯一拥有 `map -> sensor`。

保留节点名 `rm_node`，因此默认私有输出仍为 `/rm_node/rog_map/occ`，与地形分析
的既有输入接口兼容。

### 5.2 地形分析

核心入口为 `TerrainAnalysis`：`SetOdometry`、`RequestClearing` 和 `Process` 分别
接收公共 Pose、清图距离以及显式时间戳/PCL 点云，输出纯 PCL 地形点云。算法不再
订阅或发布 ROS 消息，也不自行调用 ROS 时间。

ROS1 convert 负责 `/rm_node/rog_map/occ`、`/state_estimation`、`/map_clearing`
输入以及 `/terrain_map` 输出。topic、`map` frame、四类队列和适配处理频率位于
`Interface.yaml`，体素、衰减和动态障碍物阈值位于 `TerrainAnalysis.yaml`。

## 6. 规划模块

### 6.1 世界规划

核心入口为 `WorldPlanner`，以 `SetTerrain`、`SetOdometry`、`SetGoal` 更新纯状态，
以 `Step(timestamp_ns)` 返回公共 `Path` 和 `OccupancyGrid`。A*、障碍栅格、路径复用
和稀疏化均在核心；核心不持有 NodeHandle、publisher 或 ROS 时间。

ROS1 convert 负责三类输入、两个输出和消息转换。规划调度频率、队列以及 Path/
OccupancyGrid latch 策略放在 `Interface.yaml`。航点边界文件由进程入口显式传入，
不在核心硬编码安装目录。

### 6.2 本地规划

核心入口为 `LocalPlanner`，接收公共 Path/Pose/Polygon/Twist 语义和 PCL 点云，
`Step(timestamp_ns)` 产生局部路径、速度命令与可行路径点云。路径库目录由构造配置
显式传入；算法内的控制频率、速度/加速度、避障和近目标控制参数保留在
`LocalPlanner.yaml`。

ROS1 convert 负责十类输入回调及 `/path`、`/cmd_vel_corrected`、`/free_paths`
输出。topic、frame 和队列位于 `Interface.yaml`。`sensor -> vehicle` 静态 TF
属于流程编排，由 `robot_plan_stack.sh` 读取同一 Interface 后启动
`static_transform_publisher`，核心和适配器均不重复发布。

### 6.3 地形航点探索

核心入口为 `TerrainWaypointExplorer`，维护占据网格、高度层、前沿聚类、可达性、
候选评分、分支记忆和黑名单状态；输入为纯 Pose/PCL，输出为公共 Pose 和纯调试
数据。规划频率与目标发布频率是算法行为，保留在
`TerrainWaypointExplorer.yaml`。

ROS1 convert 负责 timer、输入回调、`/way_point`、`/isgoal_vaild`、完成状态和
调试可视化发布。topic、frame、队列和 latch 策略位于 `Interface.yaml`。

### 6.4 航点发布器

核心入口为 `WaypointPublisher`，负责文件航点、RViz 追加目标、到达判定、速度、
边界和目标有效状态；只接收公共 Pose 和显式时间。waypoint/boundary 文件由进程
入口传入。

ROS1 convert 负责 Odometry、RViz goal 输入和五类输出。topic、frame、队列和
适配处理频率位于 `Interface.yaml`；到达阈值、速度和发送策略位于
`WaypointPublisher.yaml`。

探索器与航点发布器共享 `/way_point` 和 `/isgoal_vaild`。它们是两个可替换的目标
来源，`robot_plan_stack.sh` 会拒绝同时启用。

## 7. 工具与监控

### 7.1 手动回环会话导出

`SessionExporter` 核心只处理 Eigen 位姿、PCL 点云、关键帧筛选和会话文件写出。
ROS1 convert 使用 ApproximateTime 同步 Odometry/PointCloud2，并把 finalize service
映射为核心 `Finalize()`。

关键帧和图优化权重在 `ManualLoopSessionExporter.yaml`；topic、service、同步队列、
同步窗口、点云坐标语义和输出目录在 `Interface.yaml`。脚本可用
`ROBOT_DOG_SESSION_ROOT` 对输出目录做当次运行覆盖。

### 7.2 状态监控

`RobotStatusMonitorCore` 只维护时间戳、消息计数、健康状态和纯字典快照，当前时间
由调用方显式传入。ROS1 脚本负责 `rospy` subscriber、终端刷新和 RViz Marker。
展示模式、超时阈值和刷新率位于 `RobotStatusMonitor.ini`，topic 与订阅队列位于
`Interface.ini`。INI 是该 Python 模块的明确例外，不与 C++ YAML 参数混用。

## 8. 接口兼容清单

| 下游接口 | 默认 ROS1 类型 | 生产者 |
| --- | --- | --- |
| `/state_estimation` | `nav_msgs/Odometry` | FAST-LIO 或先验地图定位 |
| `/registered_scan` | `sensor_msgs/PointCloud2` | FAST-LIO 或先验地图定位 |
| `/lio/odom` | `nav_msgs/Odometry` | FAST-LIO 或先验地图定位 |
| `/lio/path` | `nav_msgs/Path` | FAST-LIO 或先验地图定位 |
| `/rm_node/rog_map/occ` | `sensor_msgs/PointCloud2` | ROG-Map |
| `/terrain_map` | `sensor_msgs/PointCloud2` | 地形分析 |
| `/global_reference_path` | `nav_msgs/Path` | 世界规划 |
| `/global_obstacle_grid` | `nav_msgs/OccupancyGrid` | 世界规划 |
| `/way_point` | `geometry_msgs/PoseStamped` | 探索器或航点发布器 |
| `/isgoal_vaild` | `std_msgs/Bool` | 探索器或航点发布器 |
| `/path` | `nav_msgs/Path` | 本地规划 |
| `/cmd_vel_corrected` | `geometry_msgs/Twist` | 本地规划 |

历史接口中的拼写 `/isgoal_vaild` 为避免破坏现有上下游而保留；新代码内部不复制
该拼写作为核心类型名。接口名称需要迁移时，应只改各模块 Interface 文件，并让
同一流程的生产者和消费者同时更新。

## 9. 配置装载顺序

所有 C++ 进程入口遵循同一边界顺序：

1. 解析并验证 runtime、Interface 和资源文件路径；
2. `ros::init`，建立全局和私有 NodeHandle；
3. 把 Interface 装入私有参数命名空间；多数模块也用同一装载器装入 runtime；
4. 用命令行资源路径覆盖仅属于本次安装/运行的参数；
5. 构造 ROS1 convert，由 convert 校验并构造纯核心；
6. 进入 spin、timer 或显式处理循环。

先验地图定位的 runtime 由纯核心 `PriorMapLocalizationConfig::LoadFromFile`
直接解析，以保证脱离 ROS 时仍使用同一份算法配置；其 Interface 仍由统一的 ROS1
YAML 装载器处理。该差异不改变 runtime/Interface 的目录与命令行顺序。

脚本总是把 runtime 与 Interface 的绝对安装路径传给进程，不依赖当前工作目录。
资源路径参数如下：

| 进程 | 额外路径参数 |
| --- | --- |
| 先验地图定位 | PCD、调试日志目录 |
| 世界规划 | waypoint 文件 |
| 本地规划 | paths 目录 |
| 航点发布器 | waypoint、boundary 文件 |
| 会话导出器 | 可选会话输出目录覆盖 |

## 10. 构建和验收

核心与 ROS1 目标都由顶层 CMake 管理。建议验收顺序：

```bash
# 纯核心
cmake -S . -B build-core -G Ninja \
  -DBUILD_ROS1=OFF -DBUILD_ROS2=OFF -DBUILD_TESTING=OFF
cmake --build build-core -j2

# ROS1 适配器与安装
source /opt/ros/noetic/setup.bash
cmake -S . -B build -G Ninja \
  -DBUILD_ROS1=ON -DBUILD_ROS2=OFF -DBUILD_TESTING=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j2
cmake --install build

# 流程资源检查
install/scripts/robot_dog_rebuild.sh --check
install/scripts/robot_dog_localization.sh --check
install/scripts/robot_plan_expv2.sh --check
install/scripts/robot_plan_path.sh --check
```

静态检查还应覆盖：核心源码无 ROS API、所有 YAML/INI 可解析、Interface 参数都有
实际读取点、安装可执行文件无缺失动态库、脚本通过 `bash -n`，并至少检查一个裁剪
组合。编译和静态检查不能替代 Livox、IMU、PCD 初值和实车控制链的硬件联调；真实
数据验收时应重点复核时间同步、点云坐标语义、TF 标定和控制方向。

## 11. 本轮验证结果

2026-08-17 在 ROS1 Noetic 环境完成以下验证：

| 项目 | 结果 |
| --- | --- |
| `BUILD_ROS1=OFF` 全核心构建 | 通过，编译命令无 `/opt/ros` 或 `ros1` 路径 |
| `BUILD_ROS1=ON` 顶层完整构建 | 通过 |
| 九个 C++ ROS1 适配器定向构建 | 全部通过 |
| 顶层 `cmake --install` | 工作区安装与全新暂存安装均通过 |
| runtime/Interface 解析 | 18 个 YAML、2 个 INI 全部通过 |
| 参数读取审计 | 所有配置叶子都有读取点；资源路径仅由命令行显式注入 |
| 动态库检查 | 九个安装后的 C++ 节点均无 `not found` |
| 启动脚本 | 源码与安装版均通过 `bash -n`，四个默认 `--check` 通过 |
| 裁剪组合 | 仅 ROG-Map + 地形分析组合通过 `--check` |
| ROS master 冒烟启动 | 十个节点完成初始化并连续存活 6 秒，无崩溃 |
| FAST-LIO 输出目录 | 使用全新路径验证递归创建及 `pose.txt` 打开成功 |

ROS master 冒烟测试没有注入传感器消息，因此不能据此声明定位精度、地图质量或
控制安全性已经完成实车验收。上述硬件和数据闭环仍需在目标机器人上执行。
