# Changelog

所有的项目变更记录都将在此文件中声明。

## [v2026.08.26] - 2026-08-26
### 新增 (Added)
- 增加 SuperLioRobot 三模式建图与重定位模块
- 增加 FAST-Calib 相机–LiDAR 标定工具

### 优化 (Changed)
- 将新增模块接入纯核心、ROS1 适配及统一构建运行框架

## [v2026.08.17] - 2026-08-17
### 新增 (Added)
- 增加公共消息结构与 ROS 接口、核心拆分说明

### 优化 (Changed)
- 将启动链模块拆分为纯核心与 ROS1 适配层，支持无 ROS 核心构建
- 统一 topic、frame、队列及外部 I/O 的接口配置边界

## [v2026.08.14] - 2026-08-14
### 新增 (Added)
- 增加 ROS1 原生定位、规划、监控及统一启动脚本
- 增加对应安装产物与迁移、快速使用文档

### 优化 (Changed)
- 将 fast-lio、先验地图定位、ROG Map 与路径规划模块接入统一 CMake 和运行时框架

## [v2026.08.02] - 2026-08-02
### 新增 (Added)
- 增加 fast-lio 对于非正装雷达数据的支持
- 增加 fast-lio 建图过程中的实时可视化
- 增加 extension/codex-session-manager 插件，用于管理 codex 的 session

### 优化 (Changed)
- 修改参数类为：配置参数类、接口参数类

## [v2026.01.15] - 2026-07-15
- CodexOpen 第一版开源方案
- 数据库 第一版制备流程：
 - tools/data_processor
 - fast_lio/lidar_odometry、map_localization
 - mapping/mapper
 - dreamview/map_center_view
