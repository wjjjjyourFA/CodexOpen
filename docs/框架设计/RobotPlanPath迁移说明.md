# RobotPlanPath 流程迁移说明

## 流程差集

原 `aa_need_to_trans/script/robot_plan_path.sh` 只启动两个 launch：

```text
robot_dog_path.launch
├── terrain analysis
├── world planner
├── local planner
├── waypointExample
├── sensor -> vehicle static TF
└── local planner RViz

nudtugv_example.launch
├── ROG-Map
└── ROG-Map RViz
```

其中 ROG-Map、terrain analysis、world planner、local planner、静态 TF 和两套 RViz
均已由此前迁移的独立模块提供。唯一缺失组件是 `waypointExample`。

## 新增独立模块

该组件按实际职责命名为 waypoint publisher，归属规划域：

```text
modules/planning/waypoint_publisher/
├── CMakeLists.txt
├── waypointExample.cpp
└── config/runtime/
    ├── WaypointPublisher.yaml
    ├── Interface.yaml
    └── data/
        ├── waypoint.txt
        └── boundary.txt

ros1/src/planning/waypoint_publisher/
└── src/run_waypoint_publisher_ros1.cpp
```

安装产物：

```text
install/bin/modules/planning/waypoint_publisher/waypoint_publisher_ros1
install/bin/config/WaypointPublisher/
```

算法参数从旧 `waypoint_example.launch` 原值迁入 `WaypointPublisher.yaml`；topic、frame
和 RViz goal 输入迁入 `Interface.yaml`。`waypoint.txt` 与 `boundary.txt` 按原文件
逐字节复制。

核心源码导入时 SHA-256 与原 `waypointExample.cpp` 一致。之后只进行了框架接口拆分：

- 将 `main` 外壳拆为可由 ROS1 adapter 调用的入口；
- 将原硬编码 topic/frame 变为同值的接口参数；
- 将两个数据文件路径交给 adapter 设置。

航点读取与追加、位置/航向到达判定、等待切换、速度发布、边界发布和
`/isgoal_vaild` 状态逻辑均未改变，也未调整任何参数值。

## 组合入口

```bash
install/scripts/robot_plan_path.sh
```

`robot_plan_path.sh` 与 `robot_plan_expv2.sh` 都是薄入口，共享
`robot_plan_stack.sh`。两者唯一的默认算法差异是：

| 流程 | terrain waypoint exploration | waypoint publisher |
| --- | --- | --- |
| `robot_plan_expv2` | 开 | 关 |
| `robot_plan_path` | 关 | 开 |

两个航点生产者都发布 `/way_point` 和 `/isgoal_vaild`。组合器会拒绝同时启用，
避免同一接口出现相互竞争的发布者。

旧流程中 world planner 与 `waypointExample` 读取同一个
`waypoint_example/data/waypoint.txt`。新 `robot_plan_path` 仍将
`WaypointPublisher/data/waypoint.txt` 同时传给二者；`robot_plan_expv2` 的 world
planner 资源绑定不变。

## 默认接口

| 方向 | topic/frame |
| --- | --- |
| 输入 | `/state_estimation` |
| RViz 追加目标 | `/move_base_simple/goal` |
| 当前航点 | `/way_point`，frame `map` |
| 航点显示 | `/way_point_show`，frame `map` |
| 期望速度 | `/speed` |
| 导航边界 | `/navigation_boundary`，frame `vehicle` |
| 目标有效状态 | `/isgoal_vaild` |

TF 沿用现有约定：定位提供 `map -> sensor`，规划组合器提供
`sensor -> vehicle`。未新增或修改 ROG-Map 的历史 TF。

## 等价性验证

- 旧 terrain analysis、world planner、local planner、waypoint launch 参数与拆分后的
  runtime/interface YAML 逐项相等；
- 旧 ROG-Map YAML 与新的 runtime + interface 合并结果相等；
- waypoint 和 boundary 安装资源与原文件逐字节相等；
- `waypoint_publisher_ros1` 已通过顶层 Ninja 编译和安装；
- `robot_plan_expv2.sh --check`、`robot_plan_path.sh --check` 和 waypoint-only
  组合检查通过；
- 同时启用两个航点发布者会被组合器明确拒绝。
