# 阿克曼室外自主导航系统 | Ackermann Outdoor Autonomous Navigation System

### 中文简介

本项目面向校园、园区等室外道路场景，开发了一套基于 ROS1 的阿克曼无人车自主导航系统。系统以 **OSM 开源路网地图和 GNSS 目标点** 为基础，不依赖高精地图，结合 FAST-LIO 激光里程计与 GNSS 定位，实现室外环境下的全局定位与点到点路径规划。

在规划层，采用 OSM 路网 A\* 实现全局路径搜索，并针对车辆起步接入、掉头及复杂返程衔接等场景，引入选择性 Hybrid A\* 搜索，提高路径的运动学可行性。通过相机道路信息与 LiDAR 路沿检测，对全局参考路径进行在线靠右修正。在局部规划层，集成 NeuPAN 实现轨迹优化与动态障碍物避让，最终结合阿克曼底盘控制、定位安全保护及运行状态监测，构建从环境感知、定位、路径规划到车辆执行的完整室外自主导航系统。

**主要技术特点：**

- 无高精地图导航：基于 OSM 路网与 GNSS 目标点实现全局导航。
- 多源定位与鲁棒性增强：FAST-LIO 与 GNSS 协同定位，GPS Guard 支持卫星信号退化时的本地里程计续推。
- 选择性 Hybrid A\*：面向特定路径连接和掉头场景进行运动学约束搜索。
- 多源道路参考修正：结合相机道路信息与 LiDAR 路沿检测，实现路径靠右修正。
- NeuPAN 局部规划：面向阿克曼车辆实现局部轨迹优化与动态障碍物避让。

### English Description

This project develops a ROS1-based outdoor autonomous navigation system for Ackermann-steered unmanned vehicles, targeting campus roads, industrial parks, and other structured outdoor environments. It adopts **OpenStreetMap (OSM) road networks and GNSS waypoints** for global navigation without relying on high-definition (HD) maps. FAST-LIO LiDAR odometry and GNSS positioning are integrated to provide localization and point-to-point navigation capabilities.

At the global planning level, A\* search on the OSM road network generates reference paths, while a selective Hybrid A\* planner is introduced for specific scenarios such as initial route connection, U-turns, and complex return-path transitions. Camera-based road information and LiDAR-based curb detection are used as optional enhancements to refine the reference path and maintain right-side driving. At the local planning level, NeuPAN performs trajectory optimization and dynamic obstacle avoidance. Combined with Ackermann vehicle control, localization safety mechanisms, and system diagnostics, the project establishes an integrated outdoor autonomous navigation framework from perception and localization to planning and vehicle execution.

**Key Features:**

- **HD-Map-Free Navigation:** Global route planning using OSM road networks and GNSS waypoints.
- **Robust Multi-Source Localization:** FAST-LIO and GNSS integration with GPS Guard for degraded GNSS conditions.
- *Selective Hybrid A Planning:*\* Kinematically feasible path generation for specific route connection and U-turn scenarios.
- **Multi-Modal Path Refinement:** Camera-based road perception and LiDAR curb detection for right-side path correction.
- **NeuPAN-Based Local Planning:** Trajectory optimization and dynamic obstacle avoidance for Ackermann vehicles.

**Framework:** ROS1 Noetic | FAST-LIO | GNSS/RTK | OpenStreetMap | Hybrid A\* | NeuPAN | Ackermann Chassis


自维护功能包的职责与目录约定见 [功能包分类](src/README.md)。

当前保留NeuPAN实车主线：雷达/LIO、GNSS、底盘、OSM、GPU视频与JSON接收、相机标定和录包。

- [核心启动指令](核心启动指令.md)
- [设备与网络配置](nuc15设备接入.md)
- [当前问题与验收](PROJECT_ISSUES_CN.md)
- [主线回归测试](src/Test/README.md)

2026-11-06已移除旧实车录包、历史测试视频、Gazebo/TEB/RVO对照及其专属测试。主车模型、地图、标定和未解决问题证据保留；`rosbags/`、`videos/`仍是后续新录制的输出位置。
