# 阿克曼无人车室外导航项目分析

> 更新日期：2026-10-28。本轮逐文件补齐整个neupan_ros包与外部NeuPAN算法的职责、改动和上游对照；定位等其他模块沿用此前`704e6fc`基线分析。
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
参考路线 + 位姿 + LaserScan → NeuPAN循环求解 → ROS外层复核/有效控制段派发 → /cmd_vel/auto
自动速度 / 云端遥控 → mux → /cmd_vel/selected → 底盘限幅与校验 → MCU
```

主线默认 `use_global_ekf=false`，使用全局定位保护桥。相机靠右控制和路沿控制默认关闭；只有显式启用后，参考路径才走对应修正分支。室外局部掉头连接默认开启，但仅满足反向路线条件时触发，并非全程替代OSM规划。

### 2.2 “跑完全程再停止”的准确含义

用户流程是：接收目标、沿路线行驶、到达后停车等待下一目标。源码中 `update_goal_arrival` 设置等待状态；主循环在等待时持续发布零速度，节点本身不退出。

行驶过程中，`run`反复读取位姿、装载路径并调用NeuPAN。室外默认启用`AcceptedExecution`：主循环接入合格解，独立派发线程约10 Hz复核当前位姿、最新完整扫描和剩余控制，求解暂时阻塞时可继续合格动作。因此：

- **任务层面**：当前路线通常连续执行到终点。
- **计算层面**：沿途多次求解，不是出发前只计算一次全部速度。
- **停车层面**：扫描故障、定位不安全、指令超时、规划器停止状态或掉头连接等待，都可能在到达前输出零速度。这属于已有保护，不意味着必须增加新的任务管理系统。

源码仍允许回调接收新目标/路径，由主循环统一装载；并没有禁止途中来消息的硬锁。但“接口有能力”不等于“用户正在使用途中换任务”。不据此要求新增任务抢占系统。当前可选相机已提供稳定参考修正，路线更新与控制续接通过已有版本/撤销机制协调。

依据：[neupan_core.py](src/neupan_ros/src/neupan_core.py) 的 `run`、`update_goal_arrival`、`goal_request_callback`、`queue_route_update`、`apply_pending_route`。

### 2.3 当前控制保护

| 机制 | 当前行为 | 适用边界 |
| --- | --- | --- |
| MCU失联停车 | 用户确认未收到速度指令会停止 | 本轮未审查MCU固件，未测具体停止时间/距离 |
| 自动源超时 | mux默认0.5秒未收到新自动指令输出零速度 | mux正常运行时生效；消息接收时间不是求解输入时间 |
| 遥控源超时 | 完整launch默认1.0秒 | 手动模式不受自动定位门控拦截 |
| 定位安全心跳 | 主线自动模式检查安全值，缺失或达到默认1.0秒期限停车 | `use_global_ekf=true` 分支关闭此门控，不能假设两分支等价 |
| 扫描保护 | 缺失、超时、转换故障及恢复稳定性检查 | 扫描按其测量时刻查询TF；发布前检查实际复核快照，不用新回调给旧结果续期 |
| 有效控制段派发 | 复核并按时间推进同一原生控制序列，版本/期限固定 | 不是第二个优化器；限速和连续性修正会影响实际动作，需与求解线程共同验证 |
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
| `neupan_ros` | 求解编排、路径装载、输入/轨迹复核、有限控制段派发及诊断 | `src/neupan_core.py`、`neupan_safety.py`、`neupan_navigation.py`、`neupan_execution.py` |
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

GNSS源时间、ROS采集时间、消息接收时间和单调时钟各有用途。定位心跳用单调时钟计龄；mux速度源超时使用ROS时间；当前普通与掉头扫描均按采集时刻查询TF，发布前核对实际复查快照。具体误差排查维护在问题文档。

## 5. 规划与可选路径分支

### 5.1 全局路线与局部动作

OSM A*生成道路参考路线，NeuPAN根据当前位姿与障碍循环计算动作。全局路线不是直接发给MCU的控制序列；`/osm_global_path_ll`是云端展示用经纬度JSON，默认简化且不改变本地控制路径密度。

### 5.2 室外掉头连接

默认室外OSM直连分支启用 `StartRouteConnector`。新路线接入方向与车头夹角至少135°时，调用独立worker和共享C++ Hybrid A*搜索前进连接；搜索等待或准入失败时停车。普通路段仍沿原路线跟踪。

旧 `outdoor_use_hybrid_astar` 是另一整段栅格分支，不等于局部掉头开关。

### 5.3 相机与路沿

路径选择优先级为相机控制路径、路沿修正路径、原始路径；同时开启多个开关不代表融合两个修正源。

相机控制默认关闭。启用后的默认`segment_reference`只为OSM提供粗略稳定参考，接受相机光学中心距右边界约1–3 m；范围内不持续追2 m。可靠单右边界可用，双侧缺失暂不生成新修正，可保持参考或采用合格OSM；不因普通观测中断新增停车。原生NeuPAN继续局部避障，独立真实道路危险仍按原空间复核恢复；旧`segment_cache`显式视觉执行合同另行检查。

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

## 8. 整个neupan_ros包：逐程序职责与改动

### 8.1 对照范围与数量

对照作者[neupan_ros固定提交b9558f3](https://github.com/hanruihua/neupan_ros/tree/b9558f3fbce4bdab356008b50b9eb6272cebef7e)，不是只比较`neupan_core.py`。排除`.git`和`__pycache__`，原包26个文件，当前128个；同路径修改4个、相同21个、新增103个、移除旧入口路径1个。旧入口`src/neupan_node.py`已迁移为`scripts/neupan_node.py`并补充异常诊断，并非删除了节点。逐文件状态和哈希见[整个包清单](tracker_logs/review_20261028/package_scope_followup/package_inventory.json)。

| 范围 | 当前文件数 | 与原版的关系 | 是否属于实车求解/控制代码 |
| --- | ---: | --- | --- |
| `src/`、`scripts/` | 6 | 核心修改1个、增加4个辅助模块、迁移并改写1个入口 | 是，具体启用条件见下表 |
| `example/lxw_car/` | 48 | 全部为本车新增 | 只有启动选中的配置/模型和节点参与；其中39个是模型及训练记录 |
| `example/lxw_car_simulation/` | 50 | 新增仿真与对照工具 | 普通实车`full_system_run.launch`没有启动这些程序 |
| `example/gazebo_limo/` | 18 | 与原版逐字节相同 | 作者示例，普通实车入口未使用 |
| 包根目录 | 6 | CMake/README/package.xml修改，另外3个相同 | 构建、依赖、说明及安装辅助 |

当前包共有22个Python程序、2个C++程序和3个Shell脚本，下面全部列出。模型、地图、世界、图片、URDF和配置另列，不能把128个文件都理解成新增控制逻辑。

### 8.2 实车直接使用的6个Python程序

下表路径相对于`src/neupan_ros/`。原核心531行，当前约2100行；核心类方法从14个增加到55个，其中41个新增、6个原方法改变、8个消息/可视化转换方法保持原逻辑。行数包含诊断与注释，不代表优化器规模；清单中的旧行数是审计基线，不作为运行契约。

| 程序 | 用途与主要接口 | 相对原版的具体变化 | 对运动的实际影响 |
| --- | --- | --- | --- |
| [scripts/neupan_node.py](src/neupan_ros/scripts/neupan_node.py) | ROS可执行入口，创建核心并调用`run()` | 从原`src/neupan_node.py`迁移；通过rospkg查找实现模块，捕获异常后保存证据并重新抛出 | 入口及异常报告；不单独计算转向 |
| [src/neupan_core.py](src/neupan_ros/src/neupan_core.py) | 输入、任务、求解和发布的编排中心 | 增加同刻扫描TF/快照、OSM路由装载、到达等待、室外连接、相机消费、运行中参数切换、候选复查/降速/有限重求解、执行段接入、停车原因和恢复 | 已超出原版的简单算法封装；可限制动作、选择参考和触发零速 |
| [src/neupan_safety.py](src/neupan_ros/src/neupan_safety.py) | 数值检查与动作连续性：`validate_solution`、`trajectory_clearance`、`continuous_solution`、`StopRegistry`等 | 原版无此模块；分开输入缺失、绑定结果过期、授权改变、候选不合格；复制算法复用的结果；管理命令历史、限速及全扫描车体扫掠 | `continuous_solution`会改变控制副本并按阿克曼运动学重新积分；最终复核这些实际待发控制。模块中的publisher包装器也参与零速历史记录，不能仅凭文件头注释认为它只读 |
| [src/neupan_navigation.py](src/neupan_ros/src/neupan_navigation.py) | `GeometryObstacleAdapter`、`MotionFeedback`及条件制动占用检查 | 原版无此模块；从完整扫描按名义车体扫掠/角度覆盖选取最多100点；DUNE输出的点、μ、λ成组重排，供NRMP最多10点；轮式位姿差分与可选编码器接口；实测边界下的方向性停车覆盖 | 改变优化器实际看到的障碍及顺序；约1 Hz位置差分只作区间均速诊断。停车覆盖须有速度、延迟、减速度、转向误差/速率/绝对值和停止余量的完整测量边界；覆盖正倒车，不假设刹车期间保持转角。旧常转角影子仅作诊断 |
| [src/neupan_execution.py](src/neupan_ros/src/neupan_execution.py) | `AcceptedExecution`接管已接受的原生有限控制序列，独立复核/派发 | 原版无此模块；约10 Hz取新位姿/扫描，按已消耗时间推进同一控制段；版本与到期时刻固定；一次复查过期最多完整重绑复查一次；显式停止/新授权可撤销；可选实测停车余量分支 | 求解暂慢时仍可派发经复核的剩余动作。过期检查与当前输入故障分类分离；两次过期且当前输入健康时可零速重算，但不新增断流恢复等待。可选例外复用同一原生预测与方向性占用，不再用全向圆域；未标定默认仍需整段合格 |
| [src/neupan_diagnostics.py](src/neupan_ros/src/neupan_diagnostics.py) | 异常栈、输入/输出数组快照与候选拒绝证据 | 原版无此模块；`CandidateEvidenceSink`用有界异步队列写候选证据；`report_failure`报告不可恢复异常 | 普通诊断采样不触发停车；核心/入口捕获异常调用`report_failure`时，它也会先尝试发零速再写盘，覆盖求解边界外异常；日志失败不能撤销停车 |

室外普通OSM分支在设置`outdoor_reference_spacing_m=0.50`后启用运动连续性和上述适配，默认`geometric_obstacle_selection=true`、`checked_execution_enabled=true`。室内、旧Hybrid A*室外分支及单独旧launch没有完全相同的参数接线，不能把某分支测试结论扩展到所有入口。`enable_adaptive_adjust`在核心类中默认true，但完整launch明确传false；现场以实际启动参数为准。

### 8.3 本车启动、参数、训练与模型

下表路径相对于`src/neupan_ros/example/lxw_car/`；这些都是相对作者ROS包新增的实车适配。

| 文件/目录 | 含义及新增内容 | 普通室外运行关系 |
| --- | --- | --- |
| [launch/full_system_run.launch](src/neupan_ros/example/lxw_car/launch/full_system_run.launch) | 整车组合：定位、扫描转换、OSM、NeuPAN、mux、日志及可选相机/路沿；传入执行段和反馈参数 | 当前主入口。`outdoor_mode=true`且`outdoor_use_hybrid_astar=false`为本次审查主线 |
| [launch/neupan_lxwcar.launch](src/neupan_ros/example/lxw_car/launch/neupan_lxwcar.launch) | 较早的本车组合，连接底盘、LIO、雷达、扫描与NeuPAN | 独立入口，不自动具备主入口的全部室外接线 |
| [launch/neupan_lxwcar_norviz.launch](src/neupan_ros/example/lxw_car/launch/neupan_lxwcar_norviz.launch) | 较早的无RViz启动组合 | 同上，不与当前主线混用验收结论 |
| [launch/outdoor_global_only_test.launch](src/neupan_ros/example/lxw_car/launch/outdoor_global_only_test.launch) | GNSS/全局路线跟踪诊断及可选NeuPAN的独立入口 | 专项测试入口，不是主线的别名 |
| [config/neupan_config.yaml](src/neupan_ros/example/lxw_car/config/neupan_config.yaml) | 为旧launch保留的默认配置，目前内容与室内配置一致 | 不作为室外调参文件 |
| [config/neupan_config_indoor.yaml](src/neupan_ros/example/lxw_car/config/neupan_config_indoor.yaml) | 本车室内几何、55步、0.3 m/s及优化权重 | 室内分支 |
| [config/neupan_config_outdoor.yaml](src/neupan_ros/example/lxw_car/config/neupan_config_outdoor.yaml) | 室外50步×0.1 s、参考0.8 m/s、0.95×0.85 m车体、轴距0.67 m、转角及变化率限制、优化权重 | 主线室外配置；50步是5 s预测时间，不等于50个障碍或固定4.5 m距离 |
| [config/neupan_train.yaml](src/neupan_ros/example/lxw_car/config/neupan_train.yaml) | 本车DUNE离线训练设置、车体几何、数据范围及训练超参数 | 离线训练使用，不在导航时持续训练 |
| [dune_train_lxw_car.py](src/neupan_ros/example/lxw_car/dune_train_lxw_car.py) | 读取专用训练配置，调用原生`train_dune()` | 手动训练入口；新增脚本不等于重写了神经网络 |
| `model/acker_robot_default_2/` | 一组本车DUNE检查点及训练记录 | 完整launch室内选`model_5000.pth` |
| `model/acker_robot_default_6/` | 另一组保留的本车检查点及训练记录 | 当前完整launch室外未选它；不可只看配置注释断言正在使用 |
| `model/acker_robot_default_7/` | 当前室外选用的DUNE检查点组 | 完整launch两个室外分支均选`model_5000.pth` |

每组模型均有`model_0.pth`、`model_500.pth`直到`model_5000.pth`共11个权重文件，以及`results.txt`、`train_dict.pkl`，三组共39个文件。它们是数据资产，不是39套控制器。文件存在不能证明训练质量或车体几何已验收；变更模型/外形时需核对训练记录和实际载入路径。

### 8.4 新增仿真目录的每个程序

以下路径相对于`src/neupan_ros/example/lxw_car_simulation/`。16个程序均为新增，普通实车主launch没有启动它们；部分脚本会被catkin安装，但“被安装”不等于“实车正在运行”。

| 程序 | 含义与用途 |
| --- | --- |
| `scripts/ackermann_sim_driver.py` | 仿真阿克曼底盘，接速度/前轮转角，积分位姿并输出里程计、关节及Gazebo模型状态 |
| `scripts/gazebo_goal_marker.py` | 将目标点显示为Gazebo可视标记 |
| `scripts/publish_demo_goal.py` | 发布演示目标 |
| `scripts/neupan_sim_controller.py` | NeuPAN仿真对照适配器；10 Hz调用算法、转换输入并发布对照所需状态；不等价于实车核心全部保护 |
| `scripts/outdoor_motion_controller.py` | 仅仿真使用的实车核心派生适配；扫描时刻TF、障碍速度输入、有限重启求解及道路/让行实验 |
| `scripts/scan_motion_estimator.py` | 仿真中按扫描簇估计障碍速度，并提供`PredictiveYieldGuard`让行实验；确有额外实验判断，但未接入普通实车入口 |
| `scripts/outdoor_simulation.py` | 独立ROS/Gazebo环境、传感器桥接、场景执行、隔离和结果记录；不启动真实串口/GNSS/雷达/GPU设备 |
| `scripts/outdoor_road_simulation.py` | 道路静态障碍、横穿及校园场景自动测试，核对真实仿真传输和车体净距 |
| `scripts/outdoor_osm_world.py` | 由本地OSM生成平面道路/建筑Gazebo场景，缺失宽度和高度采用显式场景假设 |
| `scripts/planner_comparison.py` | 构造相同参考/场景，分别运行TEB与NeuPAN并记录通过、停顿等指标 |
| `scripts/audit_comparison_transport.py` | 离线检查对照bag的时钟、扫描、里程计和几何是否匹配 |
| `scripts/report_comparison.py` | 汇总对照试验、审计、轨迹图及同步视频 |
| `scripts/export_comparison_video.py` | 导出TEB/NeuPAN时间对齐视频及指标画面 |
| `scripts/export_outdoor_road_video.py` | 导出道路仿真录像及按仿真时间匹配的中文状态画面 |
| `src/sim_pose_world_plugin.cpp` | 在Gazebo物理更新线程应用运动学模型位姿，减少SetModelState传输对仿真时钟的干扰 |
| `src/teb_sim_controller.cpp` | 调用已安装的TEB插件作仿真对照，并把角速度接口适配成本工程的转角命令；不是实车NeuPAN内部子模块 |

其余仿真文件分工如下，均未改变实车主入口的配置来源：

| 文件 | 含义 |
| --- | --- |
| `launch/lxw_car_simulation.launch` | 本车Gazebo入口的包装 |
| `launch/neupan_lxwcar_gazebo.launch` | Gazebo世界、车辆、虚拟驱动及可选NeuPAN/目标/RViz |
| `launch/outdoor_simulation.launch` | 室外OSM仿真基础组合 |
| `launch/outdoor_road_simulation.launch` | 道路场景自动回归组合 |
| `launch/planner_comparison.launch` | TEB/NeuPAN对照入口 |
| `config/neupan_config_sim.yaml` | 本车基础仿真参数 |
| `config/comparison_neupan.yaml`、`comparison_neupan_adapter.yaml` | 对照用算法及适配配置 |
| `config/comparison_teb.yaml`、`comparison_teb_field.yaml`、`teb_field_adaptation.json` | TEB对照参数与现场参数映射记录 |
| `config/outdoor_road_profiles.yaml`、`rvo_config_complex.yaml` | 道路试验组合及仿真运动障碍设置 |
| `CMakeLists.txt` | 安装仿真脚本，可用依赖存在时编译Gazebo插件/TEB适配器；缺少这些可选依赖不阻止正常NeuPAN构建 |
| `urdf/`、`meshes/`、`models/`、`worlds/`、`maps/`、`rviz/` | 车体/传感器、障碍、场景、测试地图和显示资产；完整路径见包清单 |
| `README.md`、`docs/` | 仿真运行与对照解释 |

### 8.5 原示例、构建和安装辅助

| 程序/文件 | 含义 | 相对原版 |
| --- | --- | --- |
| `example/gazebo_limo/dune_train_limo.py` | 作者LIMO模型训练入口 | 未改 |
| `example/gazebo_limo/gazebo_example_setup.sh` | 作者LIMO/RVO依赖与示例工作空间准备 | 未改；只阅读，未执行 |
| `example/gazebo_limo/run_neupan_gazebo_exp.sh` | 启动作者Gazebo场景及NeuPAN示例 | 未改；只阅读，未执行 |
| `source_setup.sh` | 将工作空间环境加入用户Shell配置的旧辅助脚本 | 未改；不由导航循环执行 |
| `CMakeLists.txt` | 安装节点与4个辅助模块，加入可选仿真构建目录 | 已改；运行入口和模块需要同时部署 |
| `package.xml` | 消息、TF、rospkg、Gazebo和跨包运行依赖声明 | 已改；增加本车链路所需依赖，不是优化目标修改 |
| `README.md` | 包的安装、配置和使用说明 | 已改 |

原`example/gazebo_limo`其余launch/config/模型/world/RViz/说明均与作者相同，完整文件名见包清单；并没有对作者全部示例重复修改。

### 8.6 不在neupan_ros目录、但直接影响本车的代码

| 程序 | 与NeuPAN的关系 |
| --- | --- |
| [osm_camera_path_refinement.py](src/osm_map_handler/src/osm_camera_path_refinement.py) | 产生相机建议参考。默认`segment_reference`：直路上约1–3 m内保持，确认持续越界后平滑修正；可靠单右边界可用，无双边界本身不停车。独立真实边界危险仍保留 |
| [osm_camera_path_guard.py](src/osm_map_handler/src/osm_camera_path_guard.py) | 消费端核对参考身份/心跳、OSM回退、独立危险及恢复；区别普通建议参考与旧显式视觉执行合同 |
| [osm_outdoor_start_connection.py](src/osm_map_handler/src/osm_outdoor_start_connection.py) | 路线准备、50 cm重采样、反向起步局部连接和室外跟踪监督；局部连接搜索不是额外的全程NeuPAN预测器 |
| [control_mode_mux.py](src/robot_base_driver/scripts/control_mode_mux.py) | 手动/自动选择、自动0.5 s新命令期限及定位安全门控 |
| [base_driver_node.py](src/robot_base_driver/scripts/base_driver_node.py) | 限幅、转角/轮速协议转换和串口审计 |
| `src/Test/scripts/`、`src/Test/tests/` | 离线复查、数据回放和回归；不是车辆运行时控制模块 |
| `/home/nuc/lxw/NeuPAN/neupan/` | 当前Noetic实际import的算法本体，详见第9节；不在catkin包内，不能用宿主机其他安装副本的测试代替 |

因此，评价“靠右是否正常”必须同时检查生产参考、消费参考和实际执行，不能只看`neupan_ros`目录是否编译成功。静态Codegraph依赖可确认核心到4个辅助模块的连接；跨包运行时Python导入、ROS remap、线程和条件launch仍以源码和实际参数核验。

## 9. NeuPAN算法：逐程序含义与修改

### 9.1 算法位置和三种比较基线

实际Noetic进程导入`/home/nuc/lxw/NeuPAN/neupan/`，不是工作空间里另一份同名目录。比较时分别使用：

| 基线 | 固定提交 | 用来回答的问题 |
| --- | --- | --- |
| 本机最初导入版本 | `947ddb0cc2b4aa1a537176109305327545b214c5` | 相对本机初始提交，现在工作树改了哪些程序 |
| 作者py38版本 | [70991f0](https://github.com/hanruihua/NeuPAN/tree/70991f0d96b7a15135ba02ea8ad3f3090ff0926e) | 与作者Python 3.8分支存在什么差异 |
| 作者main版本 | [d225c38](https://github.com/hanruihua/NeuPAN/tree/d225c3862654ade30ccd19455997f615a689a670) | 与本次获取的作者main快照存在什么差异 |

本机相对初始提交仅`neupan/blocks/initial_path.py`未提交修改；其余差异已存在于本机导入版本，不能统称为最近增加的功能，也不能未经历史追溯断言都由本项目开发。算法包内13个Python文件相对py38有8个不同，相对main有9个不同。完整哈希/状态见[算法程序清单](tracker_logs/review_20261028/package_scope_followup/algorithm_inventory.json)。

### 9.2 算法包内全部13个Python文件

表中路径相对`/home/nuc/lxw/NeuPAN/`。“本机未改”指相对`947ddb0`未改，不表示与作者版本一致。

| 程序 | 在算法中的含义 | 本机工作树改动 | 与作者固定版本的差异及运行影响 |
| --- | --- | --- | --- |
| [neupan/__init__.py](/home/nuc/lxw/NeuPAN/neupan/__init__.py) | 导出`neupan`类，供ROS封装import | 未改 | 与py38/main一致 |
| [neupan/neupan.py](/home/nuc/lxw/NeuPAN/neupan/neupan.py) | 公共入口：读YAML、建立robot/InitialPath/PAN、生成每轮参考和名义控制、调用优化、保留热启动、返回首步动作和预测；处理到达与原生stop | 未改 | 与py38/main的差异为`q_s`标量/向量说明等文档；不能由文档变化推导本机已支持向量权重 |
| [neupan/blocks/__init__.py](/home/nuc/lxw/NeuPAN/neupan/blocks/__init__.py) | 导出6个算法组件 | 未改 | 与py38/main一致 |
| [neupan/blocks/initial_path.py](/home/nuc/lxw/NeuPAN/neupan/blocks/initial_path.py) | 路点生成线段/Dubins/Reeds参考，确定路径进度/挡位/到达，按上一轮控制生成名义状态，生成本轮50步参考 | **已改** | 主路径改用弧长与`abs(ref_speed)*dt`推进，插值位置/最短航向角，处理不均匀/重复点；增加有限值、每步位移及航向连续性校验；旧圆线求交辅助增加数值容差，拒绝无前向交点却跳到远端终点。详见9.3 |
| [neupan/blocks/pan.py](/home/nuc/lxw/NeuPAN/neupan/blocks/pan.py) | PAN迭代协调器：将名义轨迹转成各时刻点流，交替调用DUNE和NRMP，按变化量提前结束 | 未改 | 本机已有`point_velocities=None`时补零的静态障碍适配；作者固定py38/main的相应实现不同，另有默认dict参数差异。这是基线差异，非本轮新增预测器 |
| [neupan/blocks/dune.py](/home/nuc/lxw/NeuPAN/neupan/blocks/dune.py) | 用ObsPointNet将障碍点编码为μ/λ，估计目标距离、排序并输出最小距离；载入或训练DUNE模型 | 未改 | 本机保留可变默认dict、未显式拒绝robot=None；缺模型/训练交互选择退出时使用`sys.exit`，作者版本采用显式异常及参数校验。模型与编码公式未因此重写 |
| [neupan/blocks/obs_point_net.py](/home/nuc/lxw/NeuPAN/neupan/blocks/obs_point_net.py) | DUNE的MLP网络结构，将二维点映射为车体边数对应的非负潜在特征 | 未改 | 与py38/main一致；新增ROS辅助代码不等于新增网络层 |
| [neupan/blocks/nrmp.py](/home/nuc/lxw/NeuPAN/neupan/blocks/nrmp.py) | 构造可微凸优化层，结合参考跟踪、避障距离变量、车体约束和线性化运动学求解控制/状态序列 | 未改 | 本机`q_s`为标量；作者版本支持3维状态权重并增加dtype转换等处理。本机没有对应向量权重实现；不能直接照作者新示例填`[qx,qy,qyaw]` |
| [neupan/blocks/dune_train.py](/home/nuc/lxw/NeuPAN/neupan/blocks/dune_train.py) | 离线生成车体距离训练数据、训练/验证网络、保存检查点及测试结果 | 未改 | 与作者主要是保存路径变量拼写和注释差异；不是新增训练方法，普通导航不运行训练循环 |
| [neupan/robot/__init__.py](/home/nuc/lxw/NeuPAN/neupan/robot/__init__.py) | 导出robot类 | 未改 | 与py38/main一致 |
| [neupan/robot/robot.py](/home/nuc/lxw/NeuPAN/neupan/robot/robot.py) | 车体顶点/半空间、阿克曼/差速/全向线性化、参考代价及速度/预测内变化率约束 | 未改 | 本机参考状态代价用标量`q_s`；作者增加向量权重分支。原生相邻控制的变化率约束已存在，ROS新增的是与上次实际发出指令/可用反馈的首步连接 |
| [neupan/configuration/__init__.py](/home/nuc/lxw/NeuPAN/neupan/configuration/__init__.py) | Torch设备、NumPy/Tensor转换、数值类型和计时开关 | 未改 | 本机转换固定float32；作者版本统一`tensor_dtype`并扩展`np_to_tensor`梯度参数。与NRMP输出转换共同构成版本差异 |
| [neupan/util/__init__.py](/home/nuc/lxw/NeuPAN/neupan/util/__init__.py) | 计时、角度归一化、坐标变换、多边形半空间、文件查找与抽点 | 未改 | 与py38一致；对main主要差在`tuple[...]`类型注解，为Python 3.8兼容差异，不是新避障方法 |

### 9.3 已修改initial_path.py究竟改变什么

以前根据平均路径间距跨点，或用圆与后续线段求交来取参考；现在每步沿路径弧长推进`|v_ref|×dt`，避免50 cm重采样与0.1 s预测混用时错误跨点。当前室外0.8 m/s、0.1 s的目标推进量为每步0.08 m，50步总参考弧长最多约4 m；终点截断、弯曲或限速时不能简单等同于固定前视直线距离。

它调整的是优化器的参考和名义输入，不直接生成最终底盘指令，也不替代OSM或局部障碍优化。`curve_style='reeds'`表示一种参考曲线生成方式；最终输出是PAN/NRMP优化后的状态与控制序列，不能理解为“每次仅输出一条不再优化的RS曲线”。

### 9.4 源文件未改，也可能改变实际算法行为

| 变化入口 | 实际改变了什么 | 没有据此改变的内容 |
| --- | --- | --- |
| ROS的`GeometryObstacleAdapter` | 在当前planner实例上包装PAN点流生成、挂DUNE输出hook，改变100/10点预算内的选点及μ/λ/点的成组顺序 | 没有改DUNE权重、MLP结构和NRMP目标函数源文件；有限选点仍是近似 |
| 室外YAML及可选自适应profile | 车体几何、时间步、预测长度、速度、控制限值和目标权重 | 改数值不等于更换优化算法；主launch自适应默认关闭 |
| 选中的`.pth` | DUNE实际推理权重 | 载入既有模型不等于本轮训练了新模型 |
| `continuous_solution`、降速和有限重求解 | 原始控制可被限幅并重新积分；必要时用不同热启动/参考速度再调用**同一个NeuPAN** | 没有新增第二套优化器；但实际下发动作确实可能不同于原生首步 |
| `AcceptedExecution` | 按时间派发已有原生控制段，并用新输入重新检查 | 不重新优化路线；不无限重复旧速度，也不刷新原控制段期限 |

正常实车只有一个NeuPAN预测优化器；外层复用阿克曼运动学作动作复核，仿真目录的障碍速度/让行实验另有独立启用条件。

### 9.5 关键参数应按公式解释

| 参数 | 本机算法中的实际含义 | 不应据此作出的推断 |
| --- | --- | --- |
| `receding`、`step_time` | 预测步数和每步时间，当前室外50×0.1=5 s | 不是`receding×ipath.interval`决定固定预测距离 |
| `dune_max_num` | DUNE输入点预算，当前100 | 不是规划步数 |
| `nrmp_max_num` | 每个预测时刻参与NRMP约束/代价的障碍点上限，当前10 | 不是“局部搜索最多10步” |
| `q_s` | 参考状态误差在平方项内的缩放，本机为标量，含位置及航向 | 不是本机可分别配置的x/y/yaw权重，也不是纯位置阈值 |
| `p_u` | 纵向速度跟踪误差在平方项内的缩放：对应`(p_u*(v-v_ref))²` | 不是直接惩罚转角变化或相邻命令变化的平滑系数 |
| `eta` | 距离变量奖励项`-eta*sum(d)`的权重，配合距离上下界与避障违约罚项 | 不是单独决定全部避障强弱的势场半径 |
| `d_min`、`d_max` | 优化中距离变量d的下界/上界，参与避障代价与约束 | 不是小于d_min才开启避障，也不是大于d_max就删除全部障碍点 |
| `robot.max_acce[1]` | 阿克曼前轮转角变化率上限，当前0.08 rad/s | 不是yaw角速度；从0到0.35 rad仅按该上限需约4.38 s |
| `collision_threshold` | 原生`check_stop()`使用的最小学习距离阈值，当前配置0.001 m | 不等于外层完整扫描的0.05 m净距要求，也不保证5 s全预测均满足该净距 |

三份本车配置的`p_u`、点数预算、预测时长、`q_s`及距离变量注释已按上述公式统一；只改说明，参数数值保持不变。参数实验及实测验收在问题文档维护，调参仍以实际加载值为准。

### 9.6 算法仓库其他全部Python程序

这些不在普通ROS实车循环中执行，相对本机初始提交均未修改。

| 程序 | 用途 | 与作者快照关系 |
| --- | --- | --- |
| 根目录`__init__.py` | 空包标记 | 与py38/main相同 |
| `example/run_exp.py` | IR-SIM运行入口，读取场景/规划参数，转换扫描、循环求解和显示 | 相对py38/main仅少一段向量`q_s`演示注释 |
| `example/dune_train/dune_train_acker.py` | 原生阿克曼DUNE训练示例 | 相同 |
| `example/dune_train/dune_train_diff.py` | 原生差速DUNE训练示例 | 相同 |
| `example/LON/LON_corridor.py` | IR-SIM走廊学习/调参试验，计算跟踪、速度、距离损失并更新部分参数 | 相同 |
| `example/LON/LON_corridor_01.py` | 使用`planner_0.1.yaml`的走廊调参试验 | 相同 |
| `example/LON/LON_corridor_02.py` | 使用`planner_0.2.yaml`及不同距离损失的走廊调参试验 | 相同 |

仓库`pyproject.toml`、`requirements.txt`负责安装/依赖，`example/*/planner.yaml`与`env.yaml`负责原生仿真；实车主线读取8.3节的本车配置。它们不自动覆盖ROS参数。

## 10. 原版已有能力与本车新增职责的边界

| 能力 | 作者原版及本机原生算法 | 当前ROS工程层 |
| --- | --- | --- |
| 预测、跟踪、避障优化 | **已有**PAN/DUNE/NRMP滚动预测与优化，输出整段控制和首步动作 | 复用，不能再以“原生不会预测”为理由另造求解器 |
| 预测内速度/转角与变化率 | **已有**运动学与预测序列相邻控制约束 | 增加与上次发送/可用反馈的首步连续性，可能改变原动作 |
| 原生碰撞停车 | **已有**`min_distance < collision_threshold`；DUNE最小距离来自当前时刻参与点的学习估计 | 另加完整扫描、车体几何和预测扫掠，检查对象与范围更大 |
| 完整扫描全预测的一票否决 | 原ROS封装没有当前这套外层门槛；不能把原生预测软代价等同于它 | 属于新增执行复核，其保守程度应单独评价 |
| 求解后精确绑定输入时效与恢复 | 原ROS扫描回调/位姿主要查最新TF，没有本车测量快照和恢复分类 | 属于新增职责；原版没有对应误分类分支，也未因此解决输入时效 |
| 求解暂慢时的独立控制段派发 | 原版在同一循环求解并发布首步动作 | 属于新增职责，有限覆盖、版本和发布同步由本工程负责 |
| 相机粗略靠右、单右边界和缺失回退 | 原版没有本项目这条链路 | 本工程新增OSM参考修正；原版不能作为该功能已实现的证明 |
| GNSS恢复/失锁、手动接管、MCU协议 | 原ROS包不提供本车整套定位/底盘实现 | 其他功能包承担，不能归为重写了NeuPAN目标函数 |

原版没有某个新增检查，既不能表述为“原版已经修好了这个检查的问题”，也不能表述为“原版完全没有避障”。当前具体缺陷、修改优先级及可交给执行模型的方案，统一见[问题文档](PROJECT_ISSUES_CN.md)。
