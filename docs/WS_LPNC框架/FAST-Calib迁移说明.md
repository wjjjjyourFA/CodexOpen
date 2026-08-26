# FAST-Calib 相机–LiDAR 标定工具迁移说明

## 1. 迁移结论

迁移源为：

```text
aa_need_to_trans/FAST-Calib
```

该目录对应上游 `hku-mars/FAST-Calib`，迁移时的提交为 `1018ecf`。目标工具已按
WS_LPNC/CodexOpen 的三层结构接入：

```text
script/fast_calib.sh                     # 运行编排
ros1/src/tools/fast_calib/               # ROS1、rosbag、消息与参数适配
tools/fast_calib/                        # 无 ROS 的标定核心
install/bin/config/FastCalib/            # 唯一运行配置来源
```

原独立 Catkin 包不再作为第二套构建系统使用。`tools/CMakeLists.txt` 直接将工具纳入
项目顶层 CMake，安装目标遵循 `install_module()` 约定。

## 2. 文件映射

| 原文件/目录 | 迁移后位置 | 说明 |
| --- | --- | --- |
| `include/common_lib.h` | `tools/fast_calib/include/fast_calib/fast_calib.h`、`tools/fast_calib/src/calibration_common.cpp` | 参数、点类型、几何检查、投影与结果保存 |
| `src/qr_detect.hpp` | `tools/fast_calib/src/qr_detector.cpp` | ArUco 标定板及相机圆心检测 |
| `src/lidar_detect.hpp` | `tools/fast_calib/src/lidar_detector.cpp` | 固态/机械式 LiDAR 圆孔检测 |
| `src/main.cpp` | `tools/fast_calib/src/single_calibration.cpp`、`ros1/src/tools/fast_calib/src/run_fast_calib_ros1.cpp` | 单场景核心与 ROS1 入口拆分 |
| `src/data_preprocess.hpp` | `ros1/src/tools/fast_calib/src/bag_loader.cpp` | 图像、rosbag 与 ROS 消息转换 |
| `src/multi_scene.cpp` | `tools/fast_calib/src/multi_scene_calibration.cpp`、`ros1/src/tools/fast_calib/src/run_multi_fast_calib_ros1.cpp` | 多场景求解与 ROS1 参数入口拆分 |
| `scripts/distance_filter_tool.py` | `tools/fast_calib/scripts/distance_filter_tool.py` | 离线 PCD 导出和 Open3D 选点工具 |
| `config/qr_params.yaml` | `install/bin/config/FastCalib/FastCalib.yaml`、`Interface.yaml` | 算法参数与 I/O/ROS 接口分离 |
| `launch/*.launch` | `script/fast_calib.sh` | 单场景、多场景和范围选取统一入口 |
| `rviz_cfg/fast_livo2.rviz` | `install/bin/config/FastCalib/rviz/FastCalib.rviz` | 调试点云显示 |

`tools/fast_calib/` 下没有 YAML、INI、JSON、TOML 或其他参数文件。所有参数与运行
配置只能位于：

```text
install/bin/config/FastCalib/
├── FastCalib.yaml
├── Interface.yaml
└── rviz/FastCalib.rviz
```

## 3. 核心与 ROS1 边界

`fast_calib_core` 只依赖 Eigen3、PCL、OpenCV/ArUco 和 Boost.Filesystem，内部不包含
`ros::NodeHandle`、publisher、rosbag 或 ROS 日志接口。

ROS1 适配层负责：

- 使用仓库公共 `codexopen_ros1_yaml_param_loader` 依次装载
  `FastCalib.yaml` 和 `Interface.yaml` 的 `fast_calib` 节；
- 从静态图像和指定 rosbag/topic 聚合点云；
- 兼容 `sensor_msgs/PointCloud2`、`livox_ros_driver/CustomMsg` 和仓库现有的
  `livox_ros_driver2/CustomMsg`；
- 根据 `PointCloud2` 是否包含 `ring` 字段选择机械式或固态 LiDAR 分支；
- 在单场景求解成功后按原流程以 1 Hz 发布调试点云。

旧版 `livox_ros_driver` 未安装时，适配层使用只用于 rosbag 反序列化的兼容消息定义；
`livox_ros_driver2` 复用仓库 `tools/data_processor/ros1_message` 中已有定义。

## 4. 保留的参数和算法

`FastCalib.yaml` 保留原活动配置的全部含义：

- 相机内参与畸变：`fx`、`fy`、`cx`、`cy`、`k1`、`k2`、`p1`、`p2`；
- 标定板：`marker_size`、`delta_width_qr_center`、
  `delta_height_qr_center`、`delta_width_circles`、
  `delta_height_circles`、`circle_radius`；
- 最少 ArUco 数量：`min_detected_markers`，原代码默认值为 `3`；
- 距离滤波：`x_min`、`x_max`、`y_min`、`y_max`、`z_min`、`z_max`。

`Interface.yaml` 保留原 I/O 参数 `bag_path`、`image_path`、`lidar_topic`、
`output_path`，并集中管理调试 frame、频率、队列和 topic。

单场景算法仍按原顺序执行：ArUco board 位姿估计、四个相机圆心计算、LiDAR
距离滤波、平面提取、圆孔提取、圆心排序、PCL SVD 求解 `T_cam_lidar`、RMSE、
彩色点云投影和结果写入。机械式 LiDAR 保留基于 ring 相邻点距离跳变的边缘提取；
固态 LiDAR 保留平面对齐、法线边界、欧式聚类和二维圆拟合。

多场景模式仍读取 `circle_center_record.txt` 中最后三个完整 block，将 12 对圆心以
等权 SVD 联合求解。

## 5. 输入、输出与 ROS topic

### 输入

- 与采集场景对应的一张相机图像；
- 包含 LiDAR 点云的 rosbag；
- `Interface.yaml` 或命令行指定的 LiDAR topic；
- 与实际相机、标定板和目标距离一致的 `FastCalib.yaml`。

图像和点云必须来自同一静态场景。单场景模式是离线读取，不订阅实时相机或 LiDAR
topic。

### 文件输出

在 `output_path` 下生成：

- `circle_center_record.txt`：每次成功单场景检测追加四对圆心；
- `single_calib_result.txt`：单场景 `Rcl`、`Pcl` 和相机参数；
- `colored_cloud.pcd`：以求得外参投影着色的点云；
- `qr_detect.png`：ArUco 与圆心检测标注图；
- `multi_calib_result.txt`：多场景联合求解的 `Rcl`、`Pcl`。

`output_path` 不存在时由核心自动创建。

### 调试 topic

默认名称与原工具一致：

```text
/qr_cloud
/center_cloud
/filtered_cloud
/plane_cloud
/aligned_cloud
/edge_cloud
/center_z0_cloud
/aligned_lidar_centers
/colored_cloud
```

消息类型均为 `sensor_msgs/PointCloud2`，默认 frame 为 `map`。名称可在唯一接口配置
`install/bin/config/FastCalib/Interface.yaml` 中修改。

## 6. 构建和安装

使用仓库原有命令：

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

若受限容器中的 `/home/ros/.ccache` 不可写，可只对构建命令禁用 ccache：

```bash
CCACHE_DISABLE=1 cmake --build build -j2
```

安装结果：

```text
install/bin/tools/fast_calib/fast_calib_ros1
install/bin/tools/fast_calib/multi_fast_calib_ros1
install/bin/tools/fast_calib/distance_filter_tool.py
install/lib/tools/fast_calib/libfast_calib_core.a
install/scripts/fast_calib.sh
```

## 7. 使用方式

先检查安装资源和实际配置路径：

```bash
install/scripts/fast_calib.sh --mode single --no-rviz --check
install/scripts/fast_calib.sh --mode multi --no-rviz --check
install/scripts/fast_calib.sh --mode distance-filter --no-rviz --check
```

使用 YAML 中的 I/O 路径运行单场景标定：

```bash
install/scripts/fast_calib.sh --mode single
```

不改 YAML，直接覆盖本次输入：

```bash
install/scripts/fast_calib.sh --mode single \
  --bag /path/to/scene.bag \
  --image /path/to/scene.jpg \
  --topic /livox/lidar \
  --output /path/to/calib_output
```

至少完成三个单场景后，在同一输出目录运行：

```bash
install/scripts/fast_calib.sh --mode multi \
  --output /path/to/calib_output \
  --no-rviz
```

自动识别消息并交互选取距离滤波范围：

```bash
install/scripts/fast_calib.sh --mode distance-filter \
  --bag /path/to/scene.bag \
  --topic /livox/lidar \
  --output /path/to/filter_output
```

无图形环境下可先验证 PCD 导出：

```bash
install/scripts/fast_calib.sh --mode distance-filter \
  --bag /path/to/scene.bag \
  --output /path/to/filter_output \
  --no-pick
```

## 8. 迁移验证记录

2026-08-26 在当前工作区完成以下验证：

1. 顶层 ROS1/Ninja 配置成功；完整 `cmake --build build -j2` 成功；
2. 另以 `BUILD_ROS1=OFF` 在独立临时构建目录成功构建
   `fast_calib_core`，确认核心不依赖 ROS；
3. `cmake --install build` 成功，两个可执行文件、辅助脚本和启动脚本均安装；
4. 三种 `--check` 均明确报告参数源为
   `install/bin/config/FastCalib/FastCalib.yaml` 和 `Interface.yaml`；
5. 临时三场景恒等外参数据通过安装后的 multi 模式运行，日志显示两个安装目录
   YAML 均成功加载，输出 RMSE 为 `0.0000 m`，结果为单位旋转和零平移；
6. `dataset/dual_mid360s_calib.bag` 的 `/lidar_front/points` 成功解析 75 条
   `livox_ros_driver2/CustomMsg`、共 1,500,000 点；
7. distance-filter 的 `--no-pick` 模式从同一 topic 成功生成包含 1,500,000 点的
   ASCII PCD；
8. 安装可执行文件的 `ldd` 未发现缺失动态库，`tools/fast_calib/` 中未发现任何
   参数/配置文件。

现有仓库没有与上述测试 rosbag 同步、且只包含目标四个 ArUco marker 的相机原图。
使用仓库文档示意图测试时检测到 8 个 marker，单场景程序按原约束拒绝继续；因此
最终外参精度、真实目标距离范围和 RViz/Open3D GUI 交互仍需使用实际同步采集数据
人工确认。
