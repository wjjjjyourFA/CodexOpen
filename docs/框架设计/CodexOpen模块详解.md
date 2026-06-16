## **CodexOpen 各模块详解**

> **modules** 下的代码，原生不属于任何通信机制框架；
> 以`ROS1`、`ROS2`等框架开发的代码，保持`modules` 的文件结构，放置在其`src`目录中；

### **(1) 传感器驱动层（Drivers）**

  **作用**：管理所有传感器，采集数据并发布到 `ROS1`、`ROS2` 等进行处理。
  
  **主要组件**：
  - **LiDAR**（Velodyne, Hesai）
  - **相机**（RGB, 深度）
  - **毫米波雷达**
  - **GPS/IMU**（GNSS 定位）
  - **CAN 总线**（底盘信息）

  相关代码：
```bash
modules/drivers/lidar/hesai/
modules/drivers/camera/
modules/drivers/gnss/
modules/drivers/canbus/
```

------

  ### **(2) 感知模块（Perception）**

  **作用**：处理来自 **相机、雷达、LiDAR** 的数据，检测 **障碍物、车道线、交通信号灯**。
  
  **主要任务**：
  1. **图像处理**（OpenCV, TensorRT）
  2. **目标检测**（深度学习模型，如 Faster R-CNN）
  3. **障碍物融合**（Radar + LiDAR + Camera）
  4. **语义分割**（车道线识别）
  5. **交通信号识别**

  相关代码：

```bash
modules/perception/camera/
modules/perception/lidar/
modules/perception/radar/
```

------

  ### **(3) 定位模块（Localization）**

  **作用**：基于 **GNSS + IMU + LiDAR** 实现厘米级高精度定位。
  
  **主要方法**：
  - **GNSS 定位**（RTK + GPS）
  - **IMU 惯性导航**
  - **LiDAR 地图匹配**
  - **EKF（扩展卡尔曼滤波）融合**

  相关代码：

```bash
modules/localization/fast-lio/
```

------

  ### **(4) 路由模块（Routing）**

  **作用**：根据高精地图，计算 **起点 → 终点** 最优路径。
  
  **主要方法**：
  - **Dijkstra 算法**（最短路径搜索）
  - **路径规划约束**（避免施工区域、单行道）

  相关代码：
```bash
modules/routing/
```

------

  ### **(5) 预测模块（Prediction）**

  **作用**：预测周围 **行人、车辆** 的运动轨迹，提供给规划模块。
  
  **主要方法**：
  1. **基于历史轨迹的贝叶斯模型**
  2. **基于 LSTM/RNN 机器学习的预测**
  3. **运动学模型（CV, CTRV, CTRA）**

  相关代码：
```bash
modules/prediction/
```

------

  ### **(6) 规划模块（Planning）**

  **作用**：基于 **高精地图 + 预测结果** 规划一条安全行车路径。
  
  **主要方法**：
  1. **采样搜索（Hybrid A\*）**
  2. **优化方法（QP, Lattice）**
  3. **基于神经网络的强化学习（RL）**

  相关代码：
```bash
modules/planning/
```

------

  ### **(7) 控制模块（Control）**

  **作用**：根据规划路径，计算 **油门、刹车、转向** 控制信号，控制车辆行驶。
  
  **控制算法**：
  1. **PID 控制**
  2. **MPC（Model Predictive Control）**
  3. **LQR（Linear Quadratic Regulator）**

  相关代码：
```bash
modules/control/
```

------

  ### **(8) 底盘通信模块（CANBUS）**

  **作用**：与 **车辆 ECU** 交互，获取底盘状态并发送控制信号。
  
  **主要功能**：
  - 解析车辆 **油门、刹车、方向盘** 数据
  - 发送控制指令

  相关代码：
```bash
modules/canbus/
```

