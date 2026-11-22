# 面向无高精地图环境的多模态融合室外无人车自主导航系统

**Multimodal-Fusion Outdoor Autonomous Navigation System for HD-Map-Free Environments**

### 中文简介

本项目面向校园、园区等室外道路场景，开发了一套基于 ROS1 的阿克曼无人车自主导航系统。系统以 OpenStreetMap（OSM）开源路网和 GNSS 目标点为基础，实现不依赖高精地图的室外点到点自主导航。通过集成 FAST-LIO 开源激光里程计算法与 GNSS 定位，构建全局与局部定位协同工作的导航框架。

在系统规划方面，基于 OSM 路网实现全局 A\* 路径搜索，并结合车辆运动学约束，设计选择性 Hybrid A\* 调用机制，仅在掉头、反向接入等特定场景触发路径搜索。针对室外道路实际行驶需求，开发相机道路参考与 LiDAR 路沿检测相结合的路径修正模块，实现全局参考路径的在线靠右修正。局部规划层集成开源 NeuPAN 规划器，实现动态障碍物避让，并结合阿克曼底盘控制、定位保护、控制仲裁与运行日志，完成室外自主导航系统的集成与实车适配。

- **无高精地图导航：** 基于 OSM 路网构建全局路径规划模块，实现 GPS 目标点驱动的室外导航。
- **定位系统集成与增强：** 集成 FAST-LIO 与 GNSS，开发 GPS Guard 定位保护机制，实现 GPS 失锁后的 LIO 续推及定位恢复后的渐进式校正。
- **选择性路径搜索：** 基于已有 Hybrid A\* 搜索实现，设计特定场景触发机制，处理掉头、反向接入及复杂路径衔接。
- **道路参考路径修正：** 开发相机道路参考与 LiDAR 路沿检测模块，实现 OSM 全局路径的在线靠右修正。
- **局部规划与底盘适配：** 集成 NeuPAN 局部规划器，完成阿克曼底盘的速度、转角控制及动态障碍物避让。
- **系统安全与验证：** 实现定位异常保护、控制仲裁、速度限幅、运行状态监测及数据记录。

### 第三方开源项目

本项目使用并集成以下开源算法及软件：

- FAST-LIO：LiDAR-Inertial Odometry.
- NeuPAN：开源局部规划器。
- Hybrid A\*：基于已有开源搜索实现进行系统集成与场景适配。
- ROS1 Noetic：机器人操作系统。

### English Description

This project develops a ROS1-based outdoor autonomous navigation system for Ackermann-steered unmanned vehicles, targeting campus roads, industrial parks, and other structured outdoor environments. Based on OpenStreetMap (OSM) road networks and GNSS waypoints, the system enables point-to-point outdoor navigation without relying on high-definition (HD) maps. Open-source FAST-LIO and GNSS positioning are integrated to establish a global-local localization framework.

For path planning, an OSM-based A\* module is implemented for global route generation. A selective Hybrid A\* invocation mechanism is designed to handle specific scenarios, including U-turns, reverse-direction connections, and complex route transitions. To improve real-world road-following performance, camera-based road references and LiDAR-based curb detection are integrated into customized path refinement modules for online right-side path correction. The open-source NeuPAN planner is integrated for local trajectory optimization and dynamic obstacle avoidance. The complete system is further adapted to an Ackermann vehicle platform with localization safeguards, control arbitration, and runtime diagnostics.

- **HD-Map-Free Navigation:** OSM-based global route planning with GNSS waypoint navigation.
- **Localization Integration and Robustness:** Integration of FAST-LIO and GNSS, with a customized GPS Guard mechanism for LIO-based continuity during GNSS degradation and gradual localization recovery.
- *Selective Hybrid A Invocation:*\* Scenario-based activation of an existing Hybrid A\* implementation for U-turns and complex route connections.
- **Road-Aware Path Refinement:** Customized camera-based road reference and LiDAR curb detection modules for online right-side path correction.
- **NeuPAN Integration and Ackermann Adaptation:** Integration of the open-source NeuPAN planner with Ackermann vehicle control for dynamic obstacle avoidance.
- **System-Level Safety and Validation:** Localization protection, control arbitration, speed limiting, runtime monitoring, and data logging.

### Third-Party Acknowledgements

This project builds upon and integrates existing open-source technologies, including FAST-LIO, NeuPAN, Hybrid A\*, and ROS1 Noetic. Original algorithms and implementations are credited to their respective authors.
