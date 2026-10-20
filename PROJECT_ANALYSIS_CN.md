# 阿克曼无人车室外导航项目分析

> 更新日期：2026-10-20；源码复核基线：`704e6fc`。
> 本文件只维护系统架构、模块职责、运行链路、关键接口及配置依据。所有问题、整改状态、测试与验收记录统一维护在 [PROJECT_ISSUES_CN.md](PROJECT_ISSUES_CN.md)。
> 日常修复bug只更新问题文档；架构、启动方式或接口契约改变时才同步更新本文。不再另外维护日期版复核或整改任务文档。

## 1. 项目定位与范围

这是基于ROS1 Noetic的阿克曼无人车室外导航工程：GNSS提供全局位置/航向参考，FAST-LIO与轮式里程计支持本地连续定位，OSM提供道路参考路线，NeuPAN循环计算局部动作，经控制选择器和串口驱动送入MCU。

用户当前采用顺序任务流程：完成当前路线、停车、再下发下一目标。室内导航、TEB对照、Gazebo仿真、开环采集和标定作为独立入口保留，不能默认与实车主线使用相同的配置和控制出口。

- `src/`：源码与功能包。
- `build/`、`devel/`：catkin构建产物，不作为业务源码修改入口。
- `tracker_logs/`：运行记录；录包配置由 `vehicle_bag_recorder` 管理。
- 外部 `neupan` Python包提供算法本体；MCU固件、GPU推理端和云端完整实现不属于本次完整审计范围。

## 2. 主线运行链路

### 2.1 主线与默认开关

主要入口：[full_system_run.launch](src/neupan_ros/example/lxw_car/launch/full_system_run.launch)。室外运行需显式设置 `outdoor_mode:=true`，文件默认仍为室内。

```text
Livox/IMU → FAST-LIO → LIO保护 ─┐
MCU轮式里程计 ────────────────┴→ 本地EKF → /odometry/local
GNSS位置/航向/质量 → GPS投影 + 全局定位保护 → /odometry/global
本地/全局里程计 → map与odom对齐

经纬度目标 → 地图目标 → OSM A* → 参考路线
参考路线 + 位姿 + LaserScan → NeuPAN循环求解 → /cmd_vel/auto
自动速度 / 云端遥控 → mux → /cmd_vel/selected → 底盘限幅与校验 → MCU
```

主线默认 `use_global_ekf=false`，使用全局定位保护桥。相机靠右控制和路沿控制默认关闭；只有显式启用后，参考路径才走对应修正分支。室外局部掉头连接默认开启，但仅满足反向路线条件时触发，并非全程替代OSM规划。

### 2.2 “跑完全程再停止”的准确含义

用户流程是：接收目标、沿路线行驶、到达后停车等待下一目标。源码中 `update_goal_arrival` 设置等待状态；主循环在等待时持续发布零速度，节点本身不退出。

行驶过程中，`run` 会反复读取位姿、检查路径和扫描、调用NeuPAN、发布当前动作。因此：

- **任务层面**：当前路线通常连续执行到终点。
- **计算层面**：沿途多次求解，不是出发前只计算一次全部速度。
- **停车层面**：扫描故障、定位不安全、指令超时、规划器停止状态或掉头连接等待，都可能在到达前输出零速度。这属于已有保护，不意味着必须增加新的任务管理系统。

源码仍允许回调接收新目标/路径，由主循环统一装载；并没有禁止途中来消息的硬锁。但“接口有能力”不等于“用户正在使用途中换任务”。本次不据此要求新增任务抢占机制。若以后启用实时相机路径修正或途中改目标，再单独评估。

依据：[neupan_core.py](src/neupan_ros/src/neupan_core.py) 的 `run`、`update_goal_arrival`、`goal_request_callback`、`queue_route_update`、`apply_pending_route`。

### 2.3 当前控制保护

| 机制 | 当前行为 | 适用边界 |
| --- | --- | --- |
| MCU失联停车 | 用户确认未收到速度指令会停止 | 本轮未审查MCU固件，未测具体停止时间/距离 |
| 自动源超时 | mux默认0.5秒未收到新自动指令输出零速度 | mux正常运行时生效；消息接收时间不是求解输入时间 |
| 遥控源超时 | 完整launch默认1.0秒 | 手动模式不受自动定位门控拦截 |
| 定位安全心跳 | 主线自动模式检查安全值，缺失或达到默认1.0秒期限停车 | `use_global_ekf=true` 分支关闭此门控，不能假设两分支等价 |
| 扫描保护 | 缺失、超时、转换故障及恢复稳定性检查 | 普通点云转换仍使用最新TF；接收新鲜度不等于所有源时间均已校验 |
| 最终输入保护 | 非有限值停车，正反向限速，转角限制，非法配置拒绝启动 | 软件上限不是硬件性能验收 |
| 相机路径监督 | 路线身份/摘要和心跳检查；失败回退原路线，无有效原路线停车 | 只有启用相机控制才影响控制选择 |
| 掉头连接准入 | 反向路线触发局部前进搜索，等待/失败停车 | 普通路线不经过相同的扫掠准入 |

## 3. 模块职责与源码导航

| 模块 | 职责 | 关键入口 |
| --- | --- | --- |
| `livox_ros_driver2` / `Livox-SDK2` | 雷达、IMU设备通信与消息输出 | `launch_ROS1/rviz_MID360.launch` |
| `FAST_LIO` | 激光惯性里程计、点云运动补偿 | `src/laserMapping.cpp`、`launch/mapping_mid360.launch` |
| `robot_base_driver` | 底盘串口、定位保护、EKF配置、控制选择和日志 | `navigation_core.launch`、`base_driver_node.py`、`global_odom_bridge.py`、`control_mode_mux.py` |
| `wheeltec_gps_driver` | NMEA与设备解状态解析 | `libnmea_navsat_driver`、G90相关launch |
| `osm_map_handler` | 目标投影、路网A*、相机路径修正和室外连接准备 | `global_astar_node.cpp`、`gps_goal_bridge.py`、`osm_camera_path_refinement.py` |
| `hybrid-a-star-annotation` | 共享C++车辆运动学搜索核心 | `hybrid_astar`包及其HYAS库 |
| `neupan_ros` | 局部优化ROS包装、路径装载、动作与到达状态发布 | `src/neupan_core.py`、`scripts/neupan_node.py` |
| `hybridnets_receiver` | 接收GPU视频与视觉结果JSON | `launch/receiver.launch` |
| `lidar_curb_refinement` | 路沿检测及可选路径修正 | `road_curb_debug.launch` |
| `usb_camera` / `FAST-Calib` | 相机输入、内外参及标定 | 各包config与launch |
| `vehicle_bag_recorder` / `Test` | 数据记录、开环与离线回归 | 录包配置及tests目录 |
| `teb_test` / 仿真example | 规划器对照与仿真 | 独立launch与仿真配置 |

路径公共Python模块通过 `osm_map_handler/setup.py` 导出；NeuPAN消费这些模块，不应复制另一份相机监督或掉头实现。ROS话题连接以launch的条件、参数和remap为准，不能只凭静态import图判断。

## 4. 定位、TF与时间约定

- 本地EKF融合 `/odom` 与 `/odometry/lio_guarded` 的平面位置和航向；轮式位姿采用差分方式。原始IMU由FAST-LIO使用，本地EKF不重复融合。
- 稳定定位分支由 `global_odom_bridge` 管理GNSS初始化、质量判断、失锁续推、运行中位置校正及恢复，并发布 `/localization_safe`。
- 默认允许合格的运行中非Fixed位置缓慢校正平移；初始定位与运行中校正采用不同条件。配置含义不能仅由“是否Fixed”概括。
- OSM、GPS投影和目标转换必须使用同一地图原点；坐标单位为米，经纬度另走云端接口。

| TF边 | 默认主线来源 |
| --- | --- |
| `map -> odom` | `map_odom_align_bridge`，用全局与本地里程计对齐 |
| `odom -> base_link` | 本地EKF；导航主线关闭底盘同名TF |
| `base_link -> body/livox_frame` | 完整launch静态外参，当前平移为0.1、0、0.35 m |
| `odom -> camera_init` | 完整launch静态单位变换 |
| `base_link -> gps_link` | 参数化天线安装TF |

切换到室内AMCL或全局EKF时，应重新核对TF发布者。`map_odom_align_bridge` 输入超时后仍广播最近变换，TF可查不等于数据新鲜。

GNSS源时间、ROS采集时间、消息接收时间和单调时钟各有用途。定位心跳用单调时钟计龄；mux速度源超时使用ROS时间；普通扫描转换用最新TF，掉头扫描另按采集时刻对齐。具体误差排查维护在问题文档。

## 5. 规划与可选路径分支

### 5.1 全局路线与局部动作

OSM A*生成道路参考路线，NeuPAN根据当前位姿与障碍循环计算动作。全局路线不是直接发给MCU的控制序列；`/osm_global_path_ll`是云端展示用经纬度JSON，默认简化且不改变本地控制路径密度。

### 5.2 室外掉头连接

默认室外OSM直连分支启用 `StartRouteConnector`。新路线接入方向与车头夹角至少135°时，调用独立worker和共享C++ Hybrid A*搜索前进连接；搜索等待或准入失败时停车。普通路段仍沿原路线跟踪。

旧 `outdoor_use_hybrid_astar` 是另一整段栅格分支，不等于局部掉头开关。

### 5.3 相机与路沿

路径选择优先级为相机控制路径、路沿修正路径、原始路径；同时开启多个开关不代表融合两个修正源。

相机控制默认关闭，接收器开启不等于视觉接管。相机控制启用后，消费端检查心跳与路线身份，失败回退原路线。默认相机中心距右边界目标为2 m，不能解释为右轮净距。

路沿检测和路沿控制是两个开关；仅运行检测不改变NeuPAN路径。

## 6. 关键接口与车辆参数

| 接口 | 类型/语义 |
| --- | --- |
| `/gps_goal_ll` | `NavSatFix`，经纬度目标 |
| `/move_base_simple/goal` | `PoseStamped`，地图目标 |
| `/osm_global_path` | `nav_msgs/Path`，地图参考路线 |
| `/osm_global_path_ll` | `String`，经纬度路径JSON |
| `/cmd_vel/auto` | NeuPAN自动动作 |
| `/cmd_vel_remote` | 云端手动动作，完整launch重映射到mux输入 |
| `/cmd_vel/selected` | mux最终选定动作，底盘驱动订阅 |
| `/vehicle/control_mode` | `UInt8`，0自动、1手动 |
| `/goal_arrived` | `String`，0/1到达状态，锁存发布 |
| `/localization_safe` | `Bool`，定位保护状态 |
| `/neupan/runtime` | 求解耗时、指令间隔与动作诊断 |
| `/motor/status`、`/motor/alarm` | 电机遥测和报警；按用户设计仅上报，不据此干预运动 |

本工程速度消息的 `linear.x` 为m/s，`angular.z` 为前轮转角rad，不能当作差速车辆角速度。串口协议为 `CMD,<轮速rad/s>,<转角deg>`。

| 参数 | 当前默认值 | 来源 |
| --- | --- | --- |
| 轴距 | 0.67 m | 实车配置与驱动 |
| 轮半径 | 0.1275 m | 驱动 |
| 正向/反向速度幅值限制 | 各1.5 m/s，可分别覆盖 | 最终底盘驱动及launch |
| 转角限制 | ±25° | 最终底盘驱动 |
| 速度死区 | 绝对值小于0.05 m/s时速度和转角归零 | 最终底盘驱动 |
| 室外参考速度 | 0.8 m/s | `neupan_config_outdoor.yaml` |
| 规划车体长/宽 | 0.95 / 0.85 m | 室外规划配置 |
| 预测配置 | 50步、每步0.1 s | 室外规划配置 |
| 最小路径半径 | 1.85 m | 室外规划配置 |

默认值是软件配置，不代表硬件性能已验收。现场以实际加载参数为准。修改外形、参考点或运动学参数时应联动核对DUNE模型和碰撞几何。

## 7. 部署与可观测性

项目使用ROS1 Noetic；2026-10-20复核时实际可用环境包括 `Neupan` Docker容器，宿主机ROS为humble。不能直接用宿主机ROS2环境替代ROS1运行环境。

- 现场操作参考 [核心启动指令.md](核心启动指令.md)。
- 录包配置参考 [vehicle_data.yaml](src/vehicle_bag_recorder/config/vehicle_data.yaml)；相机专题使用 [camera_right_test.yaml](src/vehicle_bag_recorder/config/camera_right_test.yaml)。
- 系统日志记录定位、控制及规划状态；`neupan_diagnostics.py` 在异常退出时保存数值快照与异常栈，记录诊断不等于自动恢复。
- 驱动 `/vehicle/command_audit` 发布限幅/量化后的协议命令及串口写入结果，综合日志按事件保留，并记录原始遥控输入和模式请求；写入成功不等于MCU执行。录包结束后在同目录JSON清单的 `recording_result` 核对各分包实际话题消息数。
- `/vehicle/control_mode/status`、`/gps_guard/status`、`/neupan/scan_health`、`/neupan/route_connection`、`/neupan/camera_path_guard` 分别解释控制来源、定位、扫描、掉头及视觉路径状态。

每次运行应能关联源码、外部算法依赖、模型、地图、标定和实际参数。问题复现、测试结果与验收进度只写入 [PROJECT_ISSUES_CN.md](PROJECT_ISSUES_CN.md)，不在本文维护第二套清单。
