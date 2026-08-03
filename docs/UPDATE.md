## 2026 更新进度：

- [x] 将代码迁移到 默认编译标准**C++14**，**CMAKE VERSION 3.16**
- [x] 代码块里不再放置 `readme.md` / `README.md`，使用 `obsidian`进行双向链接管理模块文档
- [x] 系统版本更新 `ubuntu 20` ==> `ubuntu 22`
- [x] `ROS` 通信接口更新 `neotic` && `foxy` ==> `humble`
- [x] 优先更新算法， 工程化的 CyberRT 兼容并不关键；`component` <==> `ros1_convert` 、`ros2_convert`
- [x] 添加对 `cmake` 的支持，现在只需要在镜像中执行以下语句
    `cd CodexOpen/build && cmake -G Ninja .. && ninja -j 12`
  - [x] 测试 CyberRT 基础功能：-DBUILD_TESTING=ON
  - [x] 编译 CodexOpen 不同 ROS 版本代码：-DBUILD_ROS1=ON

### docker 镜像

- [x] docker 完善、镜像优化：
  - [x] `Dockerfile` 的完善、`installers`安装脚本的完善
  - [x] 时间切换到东八区 `UTC-8`
- [x] docker [镜像下载链接](https://pan.quark.cn/s/5695784f119e)：提取码：[E3Qp]
    - [x] `jojo_ros1_noetic.tar`：x86 全面的 ROS1 国内源支持；
    - [x] `jojo_ros2_humble`：x86 全面的 ROS2 国内源支持；使用 `quick_autoware_install.sh` 安装后可直接编译 `autoware` 源码；

### modules-driver 模块

- [x] 完成 `driver-camera` | `driver-lidar` 的程序归纳 
  - [x] `drivers/camera` 更改为使用 `opencv -- cv::Mat`

### modules-perception 模块

- [x] 冻结坐标系
  - [x] 数据关联队列
  - [x] 基于`odom`的线性插值变换
    - [x] 基于`IMU`的线性插值变换：涉及外参变换；
  - [x] 提供函数实现`odom` 和 `gnss` 的坐标变换 （2D）

- [ ] 添加 `MetaDataPreprocess` 多种传感器数据的多线程预处理
  - [ ] 将相机检测的目标与雷达数据进行匹配
    - [ ] 通常通过空间校准（如对齐到同一坐标系）
    - [ ] 数据关联算法（如匈牙利算法或卡尔曼滤波）实现
  
- [x] 相机图像去畸变
  - [x] 增加鱼眼相机去畸变
- [x] 点云投影图像
  - [ ] 基于原始图像的投影计算；

- [x] 添加 图像目标检测 ` [camera_detection_single_stage](modules/perception/camera_detection_single_stage) `
  - [x] 图像 目标检测器（YOLOV5、YOLOV8）
    - [x] 官方预训练模型的部署全流程
- [x] 图像 目标跟踪器（ByteTracker、DeepSort）
  - [ ] 融合输出结果，对于经过跟踪器后，能够生成 trackID 的object，在融合结果中，为其增加 trackID；没有的，予以保留；
  
- [x] 添加 图像目标定位
  - [x] 检测与跟踪 双模式基础版
  - [ ] 全局轨迹累积：
    - [ ] 对静态目标，使用双阈值（进入/退出阈值）抑制位置抖动，提升轨迹稳定性
    - [ ] 对动态目标，利用BEV卡尔曼滤波，消除轨迹抖动，增加速度与运动方向估计，用于轨迹关联与运动预测
  - [ ] 增加利用目标先验信息过滤误检：用已知大小的 bbox ，使用**定位获得的深度**将之投影到图像后，计算 IOU 过滤误检

- [ ] 添加 图像语义分割
- [ ] 添加 地表路面材质识别
  - [ ] 附着力分析

- [ ] 360影像 || 540全景环视；
  - [ ] 无人平台环视相机标定
  - [ ] 图像到图像的单应性变换 ==> bev 图
  - [ ] 可见光图像 和 红外图像的 叠加 ==> 双光相机标定

- [x] ColorMap；
  - [x] SimilarityMap；
  - [ ] AutoEncoder；==> libtorch inference

- [x] 点云动态补偿
  - [x] 基于 IMU | POSE 的线性插值：涉及外参变换；
  - [x] 使用 fast-lio 的 IMU 预积分进行伪补偿；
- [ ] 点云地面分割 （GroundRemove）
  - [x] 简易算法，基于 高度 的粗略滤除
  - [ ] 高斯拟合 + 线性拟合 + 退化处理
  
- [ ] 添加 点云目标检测（PointPillar、 CenterPoint）
  - [ ] 动态目标滤除
  - [ ] 激光雷达 动态目标检测器
  - [ ] 点云 目标跟踪器
  
- [ ] 添加 点云语义分割

- [ ] 添加 毫米波雷达 目标检测
  - [ ] 毫米波雷达 动态目标检测器

- [ ] BEV 的目标跟踪
  - [ ] 添加 意图识别 | 行为预测

- [ ] 多传感器融合的目标跟踪
  - [ ] 设计目标融合策略（相信谁呢？）多判据方案

### modules-localization 模块

- [x] SLAM 里程计
  - [ ] LeGO-LOAM：提纯算法核心，去除 ROS1 部分
  - [x] Fast-LIO：提纯算法核心，去除 ROS1 部分
  - [ ] LIVO：提纯算法核心，去除 ROS1 部分
  - [x] 定位输出：
      - [x] Fast-LIO：xyz rpy && undistort PointCloud

- [ ] KissICP

- [x] 先验地图定位：
  - [x] 基于 Fast-LIO，和先验高精点云地图的实时定位：预测与校准
    - [x] 起始点必须在地图内，否则首帧自动匹配错误；
    - [x] 超出先验地图时，自动利用先验地图边界点云，融合 `current_scan`，退化为 Fast-LIO 进行 SLAM；
      - [ ] 拟修改为：双树地图匹配策略

### modules-mapping 模块

- [ ] 建图 mapper
  - [x] 3维建图
    - [x] 彩色化地图 
  - [ ] 闭环优化
  - [ ] 地形建模
  - [ ] 地图变化检测、增量更新
- [x] 基于地图的 `TerrainMap` 生成
  - [x] 基于 `map2d` 使用 `CVAT` 标注道路边界、可通行区域等；
    - [x] 利用区域生长和`opencv`算法，自动补全**闭区间**的语义信息

### modules-tools 模块

- [x] 相机内参标定
  - [x] 输出等效针孔模型内参数（强制保持相机原始物理参数）
  - [x] 图像 resize 后，计算新的内参数，并更新外参数。
  - [x] 鱼眼相机标定（K1K2K3K4）
      - [ ] 其他型号相机的标定实操中，二维码标定板，可识别二维码，但无法获取到匹配点，无法进行标定
  - [x] 修改涉及投影矩阵的程序。如果不是直接使用投影矩阵计算，那么需要使用内参数计算投影矩阵。

- [ ] 相机外参标定
  - [ ] 手动选点标定方法
  - [ ] 基于原始图像进行标定；不去畸变；
    - [ ] 支持鱼眼相机模型

- [ ] 雷达外参标定
  - [ ] 使用 fast-lio 算法的 LIDAR-IMU 标定

- [ ] 自动标定 AutoCalib
  - [x] 前置条件：图像检测模型、点云检测模型、多模态匹配模型
  - [ ] 输入：未去畸变图像 和 真实相机内参 || 去畸变图像 和 等效针孔模型内参
    - [ ] 检查 输入的 内参矩阵定义 是否为 严格去畸变
  - [ ] 修改输出结果为 RT 矩阵 单位 m；（避免投影再计算单位）
  - [x] 将网络模型等大数据，放在 install 文件夹中，方便随时更换或剔除；
  
### tools 模块

#### data_label 模块

- [x] 添加基于`ostrack`的跟踪类自动标注工具  `[auto_label](tools/data_label/auto_label)`
  - [x] ostrack 的跟踪输入切换为**图片**，输出切换为**时间戳文件**
  - [x] 添加序列指示器，优化画框指令、标注操作流程
  - [x] 设置选框模式；跟踪标注模式；添加`倒退\前进\模式切换`按钮；
  - [x] 添加手动配置的画面放大系数；标注格式：读取后先缩放，存储前先还原；
  
- [x] 添加 海岸线 手动标注工具 `[line](tools/data_label/manual_label/line)`
- [x] 添加 目标框 手动标注工具  `[box](tools/data_label/manual_label/box)`
- [ ] 添加 BEV视角下的自动跟踪标注工具 

#### az_toolkit 模块

- [x] 基础定义 `az_toolkit` 数据库分析工具包
- [x] **az_toolkit** `python`打包 | `github` 更新
- [x] 基于跟踪的自动标注工具  `[auto_label](tools/data_label/auto_label)`

### third_tools 模块

#### OpenMMLab 模块

- [ ] openmmlab 环境的升级：pytorch 1.13 ==> 2.0.1
- [x] mmpose 引入 yolo 的目标检测，放弃 rtmdet 模型；
- [x] mmaction 引入 yolo 的目标检测

### 数据集构建
- [ ] 数据集定义，专注于越野场景
- [ ] 目标类别定义，专注于特殊目标
- [ ] 整理数据库列表，按任务、时间、地点、天气、场景划分
- [x] 数据库组织：
  - 原生数据：`rosbag`、`waymo`等
  - 解析后的数据：`output`
  - 参数文件：
  - 说明文档：
    - 传感器类型 + 数目 + 位置 
    - 数据场景简述
  
- [ ] 数据标注：
  - 图像的2d目标框 
  - 点云的3d目标框
  - 点云的地形语义
  - 点云的像素级语义：
    - 每个点的时间戳
    - 每个点的属性

- [x] 添加 数据解析工具 ` [data_processor](tools/data_processor) ` 
  - [x] 支持离线通信接口调试： ROS1、ROS2、DDS
- [x] 添加 数据回放工具  ` [data_loader](tools/data_loader) `
  - [x] 支持离线单步调试
  - [x] 支持离线通信接口调试： ROS1、ROS2、DDS

#### 评测代码

- [ ] 里程计：相对位姿精度、绝对位姿精度
- [ ] 目标检测：
- [ ] 语义分割：
- [ ] 目标跟踪：
- [ ] 4D instance：
