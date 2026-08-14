# RobotPlanExpv2 拆分迁移说明

## 迁移结论

`robot_plan_expv2` 只是一个历史流程名，不再作为单一算法模块。其五个
程序已按数据语义拆分：

| 业务域 | 核心位置 | ROS1 适配位置 |
| --- | --- | --- |
| 感知 | `modules/perception/rog_map` | `ros1/src/perception/rog_map` |
| 感知 | `modules/perception/terrain_analysis` | `ros1/src/perception/terrain_analysis` |
| 规划 | `modules/planning/world_planner` | `ros1/src/planning/world_planner` |
| 规划 | `modules/planning/local_planner` | `ros1/src/planning/local_planner` |
| 规划 | `modules/planning/terrain_waypoint_exploration` | `ros1/src/planning/terrain_waypoint_exploration` |

每个目录有自己的 CMake 目标和 `config/runtime`，不再共享流程级
`RobotPlanExpv2/Interface.yaml`。顶层脚本 `script/robot_plan_expv2.sh` 只提供默认
全组合和可选组件开关。

## 安装目标

```text
install/bin/modules/perception/rog_map/rog_map_ros1
install/bin/modules/perception/terrain_analysis/terrain_analysis_ros1
install/bin/modules/planning/world_planner/world_planner_ros1
install/bin/modules/planning/local_planner/local_planner_ros1
install/bin/modules/planning/terrain_waypoint_exploration/terrain_waypoint_exploration_ros1
```

对应配置分别安装到：

```text
install/bin/config/RogMap
install/bin/config/TerrainAnalysis
install/bin/config/WorldPlanner
install/bin/config/LocalPlanner
install/bin/config/TerrainWaypointExploration
```

## 组合方式

默认保持原流程的五节点组合：

```bash
install/scripts/robot_plan_expv2.sh
```

例如，只使用感知部分：

```bash
ROBOT_PLAN_ENABLE_WORLD_PLANNER=false \
ROBOT_PLAN_ENABLE_LOCAL_PLANNER=false \
ROBOT_PLAN_ENABLE_TERRAIN_EXPLORER=false \
ROBOT_PLAN_ENABLE_STATIC_TF=false \
ROBOT_PLAN_LOCAL_RVIZ=false \
install/scripts/robot_plan_expv2.sh
```

所有组合开关、独立启动命令和配置覆盖方式见
[使用手册](../WS_LPNC框架/使用手册.md)。

## 消息链和 TF

```text
/state_estimation + /registered_scan
                 │
                 ├──> ROG-Map ──> /rm_node/rog_map/occ
                 │                           │
                 │                           v
                 │                    terrain analysis ──> /terrain_map
                 │                                             │
                 └──> terrain waypoint exploration         └──> world planner
                                      │ /way_point                    │
                                      └──> local planner <──────────┘
                                                  │
                                     /path + /cmd_vel_corrected
```

- 定位主树是 `map -> sensor`；
- 启用本地规划时，编排层默认补充 `sensor -> vehicle`；
- ROG-Map 的历史 `world/body/drone` frame 保持不变；
- 没有标定依据时不增加伪造的 `map -> world` TF。

## 核心算法不变

本次只移动核心 C++ 文件，并重建了外部目录、CMake、参数装载和运行编排。
移动前后的核心源码 SHA-256 逐文件一致；未修改公式、参数值、搜索逻辑和
地图更新逻辑。

完整的分层规则见 [框架结构说明](../WS_LPNC框架/框架结构说明.md)。
