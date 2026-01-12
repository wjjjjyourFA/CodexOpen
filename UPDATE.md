## 2026 更新进度：

- [x] 将代码迁移到 默认编译标准C++14
- [x] 代码块里不再放置 `readme.md` / `README.md`，使用 `obsidian`进行双向链接管理模块文档
- [x] 系统版本更新 `ubuntu 20` ==> `ubuntu 22`
- [ ] `ROS` 通信接口更新 `neotic` && `foxy` ==> `humble`

### docker 镜像
- [ ] docker 完善、镜像优化：
  - [ ] `Dockerfile` 的完善、`installers`安装脚本的完善
  - [ ] 时间切换到东八区 `UTC-8`

### modules-driver 模块
- [x] 完成 `driver-camera` | `driver-lidar` 的程序归纳 
  - [x] `drivers/camera` 更改为使用 `opencv -- cv::Mat`

### modules-perception 模块
- [x] 冻结坐标系
  - [x] 数据关联队列
  - [ ] 基于`odom`的线性插值变换
  - [x] 提供函数实现`odom` 和 `gnss` 的坐标变换 （2D）

- [ ] 添加 `MetaDataPreprocess` 多种传感器数据的多线程预处理
  - [ ] 
  
- [ ] 相机图像去畸变
- [ ] 点云投影图像

- [x] 添加 图像目标检测 ` [camera_detection_single_stage](modules/perception/camera_detection_single_stage) `
  - [x] 图像 目标检测器（YOLOV5、YOLOV8）
    - [x] 官方预训练模型的部署全流程

- [x] 添加 图像目标定位
  - [ ] 全局轨迹累积

- [ ] 添加 图像语义分割

- [ ] 360影像 || 540全景环视；
  - [ ] COLORMAP；
  - [ ] 图像到图像的单应性变换 ==> bev 图
  - [ ] 可见光图像 和 红外图像的 叠加 ==> 双光相机标定

- [ ] 点云动态补偿
- [ ] 点云地面分割 （GroundRemove）
  
- [ ] 添加 点云目标检测 （from FH ？）
  - [ ] 动态目标滤除
  - [ ] 激光雷达 动态目标检测器

- [ ] 添加 点云语义分割

- [ ] 添加 毫米波雷达 目标检测
  - [ ] 毫米波雷达 动态目标检测器

- [ ] BEV 的目标跟踪

- [ ] 多传感器融合的目标跟踪
  - [ ] 设计目标融合策略（相信谁呢？）

- [ ] 添加 BEV 跟踪模块
  - [ ] 添加 意图识别 | 行为预测

- [ ] 添加 地表路面材质识别
  - [ ] 附着力分析

- [ ] 里程计
  - [ ] LIO + LO + LIVO
  - [ ] 定位
  
- [ ] 建图 mapping
  - [ ] 闭环优化
  - [ ] 地形建模
  - [ ] 地图变化检测、增量更新

### modules-tools 模块
- [ ] 相机标定
  - [x] 鱼眼相机标定（K1K2K3K4）
  - [x] 输出等效针孔模型内参数（强制保持相机原始物理参数）
  - [x] 图像 resize 后，计算新的内参数，并更新外参数。
  - [ ] 修改涉及投影矩阵的程序。如果不是直接使用投影矩阵计算，那么需要更新内参数获取投影矩阵。

- [ ] 自动标定 AutoCalib
  - [ ] 输入：未去畸变图像 和 真实相机内参 || 去畸变图像 和 等效针孔模型内参
    - [ ] 检查 输入的 内参矩阵定义 是否为 严格去畸变
  - [ ] 修改输出结果为 RT 矩阵 单位 m；（避免投影再计算单位）
  - [ ] 将网络模型等大数据，放在 install 文件夹中，方便随时剔除；
  
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

### 数据集构建
- [ ] 数据集定义，专注于越野场景
- [ ] 目标类别定义，专注于特殊目标
- [ ] 整理数据库列表，按任务、时间、地点、天气、场景划分
- [ ] 数据库组织：
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
  - [ ] 支持离线通信接口调试： ROS1、ROS2、DDS
- [x] 添加 数据回放工具  ` [data_loader](tools/data_loader) `
  - [ ] 支持离线单步调试
  - [ ] 支持离线通信接口调试： ROS1、ROS2、DDS

#### 评测代码
- [ ] 里程计：相对位姿精度 绝对位姿精度
- [ ] 目标检测：雷达ap([x]waymo)
- [ ] 语义分割：miou
- [ ] 目标跟踪：
- [ ] 4D instance：
