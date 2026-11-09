# 阿克曼无人车室外导航项目分析

> 更新日期：2026-11-06。本次同步清理范围：移除旧录包及Gazebo/TEB/RVO对照；保留实车主线。逐程序/上游数量对照仍为此前历史审查快照。
> 待修改事项统一维护在 [PROJECT_ISSUES_CN.md](PROJECT_ISSUES_CN.md)；详细运行与测试证据保存在`tracker_logs/`，不把已完成修复过程留作问题待办。
> 日常修复bug只更新问题文档；架构、启动方式或接口契约改变时才同步更新本文。不再另外维护日期版复核或整改任务文档。

## 1. 项目定位与范围

这是基于ROS1 Noetic的阿克曼无人车室外导航工程：GNSS提供全局位置/航向参考，FAST-LIO与轮式里程计支持本地连续定位，OSM提供道路参考路线，NeuPAN循环计算局部动作，经控制选择器和串口驱动送入MCU。

用户当前采用顺序任务流程：完成当前路线、停车、再下发下一目标。室内导航、开环采集和标定作为独立入口保留，不能默认与实车主线使用相同的配置和控制出口。Gazebo/TEB/RVO对照已按用户要求清理。

**2026-10-29已实施精简：** 普通导航恢复“NeuPAN原生求解→数值/输入时效/任务归属校验→直接发布首个动作”。ROS不再改写整段控制或热启动，不再施加持续减速目标、二次全预测碰撞门槛或独立派发线程。原生选点与排序恢复，本车参数、模型、外部initial_path时间/距离修复和定位/扫描/人工停止保护保留。相机仅做约1–3 m稳定参考修正；具体现场验收仍见问题文档。

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
参考路线 + 位姿 + LaserScan → NeuPAN原生求解 → 输入时效/数值/任务归属校验 → 首动作直发 → /cmd_vel/auto
自动速度 / 云端遥控 → mux → /cmd_vel/selected → 底盘限幅与校验 → MCU
```

主线默认 `use_global_ekf=false`，使用全局定位保护桥。相机靠右控制和路沿控制默认关闭；只有显式启用后，参考路径才走对应修正分支。室外局部掉头连接默认开启，但仅满足反向路线条件时触发，并非全程替代OSM规划。

### 2.2 “跑完全程再停止”的准确含义

用户流程是：接收目标、沿路线行驶、到达后停车等待下一目标。源码中 `update_goal_arrival` 设置等待状态；主循环在等待时持续发布零速度，节点本身不退出。

行驶过程中，`run`反复绑定位姿与扫描、装载参考并调用NeuPAN，直接发布本次原生首动作。没有派发旧控制序列的第二线程。因此：

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
| 原生首动作直发 | 单一求解/发布主线；不复用过期动作 | 慢求解超过mux的0.5 s自动指令期限仍可能停车，不放宽看门狗 |
| 最终输入保护 | 非有限值停车，正反向限速，转角限制，非法配置拒绝启动 | 软件上限不是硬件性能验收 |
| 相机路径监督 | 路线身份/摘要和心跳检查；普通建议参考暂失时保持同任务已接受修正；无原路线仍停车 | 只有启用相机控制才影响控制选择 |
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
| `neupan_ros` | 原生求解编排、参考装载、输入/数值校验与诊断 | `src/neupan_core.py`、`neupan_safety.py`、`neupan_navigation.py` |
| `hybridnets_receiver` | 接收GPU视频与视觉结果JSON | `launch/receiver.launch` |
| `lidar_curb_refinement` | 路沿检测及可选路径修正 | `road_curb_debug.launch` |
| `usb_camera` / `FAST-Calib` | 相机输入、内外参及标定 | 各包config与launch |
| `vehicle_bag_recorder` / `Test` | 数据记录、开环与离线回归 | 录包配置及tests目录 |

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

定位桥在`/gps_guard/status.reference_frame`中发布`instance_id`（桥进程实例）、`odom_epoch`（既有故障重新锚定时递增）和`last_reset`（最近重锚时间与原因）。正常GPS对齐校正不推进该世代；GNSS报文会话也不是坐标世代。相机结合新鲜状态与既有`/gps_guard/local_fault_event`使真正失效的旧参考退出，对同一次事件去重；普通对齐只重投影或刷新观测，保持已接受map参考。这些字段用于参考来源判断，不另设速度控制或常态停车条件。

GNSS源时间、ROS采集时间、消息接收时间和单调时钟各有用途。定位心跳用单调时钟计龄；mux速度源超时使用ROS时间；当前普通与掉头扫描均按采集时刻查询TF，发布前核对实际复查快照。具体误差排查维护在问题文档。

## 5. 规划与可选路径分支

### 5.1 全局路线与局部动作

OSM A*生成道路参考路线，NeuPAN根据当前位姿与障碍循环计算动作。全局路线不是直接发给MCU的控制序列；`/osm_global_path_ll`是云端展示用经纬度JSON，默认简化且不改变本地控制路径密度。

室外主线的经纬度目标先转为map坐标，再由A*吸附到道路图边；默认严格道路模式不在末尾追加原始点击/GPS坐标。车辆实际定位仍来自定位链路，规划起点吸附与RViz吸附标记不改写`/odometry/global`。普通道路任务保留目标道路/方向/纵向位置，实际到达点已允许随目标段参考修正；不重发导航目标，使用原生到达函数和现有0.5 m阈值，详见问题文档1.2。

### 5.2 室外掉头连接

默认室外OSM直连分支启用 `StartRouteConnector`。新路线接入方向与车头夹角至少135°时，调用独立worker和共享C++ Hybrid A*搜索前进连接；搜索等待或准入失败时停车。普通路段仍沿原路线跟踪。

旧 `outdoor_use_hybrid_astar` 是另一整段栅格分支，不等于局部掉头开关。

### 5.3 相机与路沿

路径选择优先级为相机控制路径、路沿修正路径、原始路径；同时开启多个开关不代表融合两个修正源。

相机控制默认关闭。启用后的默认`segment_reference`只为OSM提供粗略稳定参考，接受相机光学中心距右边界约1–3 m；范围内不持续追2 m。可靠单右边界可用，双侧缺失暂不生成新修正，保持同任务已接受参考，不默认回到零偏移OSM；尚无修正则明确标记未建立。普通参考边界/区域/形状失败不创建危险历史或停车，不进入旧视觉执行恢复合同。原生NeuPAN继续局部避障，已有独立真实危险保留并按其来源处理，不能由普通参考成功清空；旧`segment_cache`显式合同与普通主线分离。

2026-10-30参考接口补充：合格候选可从实测位姿/航向作C2连接。`final_path.reference_connection`携带起点索引，由路径摘要/版本及消费身份绑定；装载适配裁掉未追踪旧前缀，加载检查过的后缀，回到完整OSM时重新定位进度。该变化只涉及参考生成/装载，不改NeuPAN求解或首动作。独立`/camera_right/processing_health`报告处理阶段/异常/最近发布帧，系统日志与录包配置已接入，不作为新停车信号。

职责是“按可靠物理右路缘确定道路段的大致侧向偏移，随后保持”，覆盖直路和普通弯路；弯路沿OSM局部法向偏移，局部障碍由NeuPAN避让。用户反馈原始OSM道路线偏移明显，因此OSM主要保留目标分支与路线连接关系，不作为观测不足时自动归零偏移的可靠回退。普通参考已取消完整视觉走廊硬否决；路口/急弯保持已有连续参考，不因标签回零。替换时同步提交过渡与连接状态；异步路径/状态配对期间消费端保持最后接受参考。真实任务/定位框架重置需要失效旧参考。原对齐门限误清参考已修复：正常校正保留map参考，既有故障重新锚定/定位桥重启才通过明确世代使旧参考失效；实车闭环仍待核对。末端回零已取消，普通道路任务末点可随目标段已接受修正一起调整，相关身份绑定和到达验收见问题文档1.2。真实边界含义和实车效果仍需核验，不能将临时可见区域的边缘变化等同于物理路缘变化。

当前实现的参考保持、退出与恢复如下；停止保护和参考退出是不同的事件。

| 条件 | 当前参考行为 | 后续处理 |
| --- | --- | --- |
| 遮挡、缺边界、普通候选失败、短时处理异常 | 同任务内保持已接受修正 | 观测恢复后继续评估，无需先退回原线 |
| 普通弯路、路口标签、临时车头偏离 | 普通弯路可新增修正；路口/临时偏航保持已有参考，不按标签回零 | 前方证据合格时评估下一次段级调整 |
| 心跳超过默认3 s或路径/状态短暂未配对 | 已接受的同任务完整配对保留 | 新配对齐全后替换；一般失联不单独清空该建议参考 |
| 普通map→odom对齐变化 | 保持map中已接受参考，重投影/必要时仅刷新odom观测；旧平滑判据仅诊断 | 不回零、不增加锁停，参考版本与连接保持 |
| 定位桥既有故障重新锚定或进程更换 | 通过reference_frame世代/已有故障事件明确失效旧参考 | 保留原定位及独立危险保护；状态/事件去重后重建，不把GNSS会话当坐标世代 |
| 接近同段修正末点 | 已取消任务末端回零，保持已接受目标段偏移 | 按实际装载末点判断到达；已完成任务不被迟到参考重新激活 |
| 新原路线时间戳/几何、重启参考节点或规划节点 | 原保持状态重建，旧任务参考不保证继承 | 基于当前任务重新建立修正；相同签名的重复路线不触发生产端重置 |
| 明确关闭靠右控制 | 消费端允许选择原OSM | 重新启用后按实际参考建立状态处理；关闭靠右不等于下发人工停止 |
| 独立危险、定位/扫描失效、人工停止、原生到达/停止 | 按各自保护处理，不能等同于普通相机参考失败 | 各来源解除/恢复，普通参考成功不清除其他保护 |

末点随参考修正的接口已同步：原OSM的`route_key()`和加密前`original_path_digest`仍绑定原任务；普通advisory候选验证原摘要、frame/stamp、候选摘要及版本，不再要求其末点等于原OSM。装载回执区分原末点、所选末点及装载末点，原生NeuPAN使用装载路径末点判断到达。同任务更新重新匹配几何位置，不把原弧长/索引生搬到新曲线；到达后排队或迟到更新不复位完成状态。只在已确认道路段内新增偏移，已知路口/不连续路段后保留原有下游参考，终点所在段按当地证据再评估，不将出发段偏移套用所有后续道路。软件离线验证不替代实车验收，1.1对齐误清参考已完成软件修复，待实车验证。

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

- 现场启动参数以[full_system_run.launch](src/neupan_ros/example/lxw_car/launch/full_system_run.launch)及进程实际加载的参数为准。
- 录包配置参考 [vehicle_data.yaml](src/vehicle_bag_recorder/config/vehicle_data.yaml)；相机专题使用 [camera_right_test.yaml](src/vehicle_bag_recorder/config/camera_right_test.yaml)。
- 系统日志记录定位、控制及规划状态；`neupan_diagnostics.py` 在异常退出时保存数值快照与异常栈，记录诊断不等于自动恢复。
- 驱动 `/vehicle/command_audit` 发布限幅/量化后的协议命令及串口写入结果，综合日志按事件保留，并记录原始遥控输入和模式请求；写入成功不等于MCU执行。录包结束后在同目录JSON清单的 `recording_result` 核对各分包实际话题消息数。
- `/vehicle/control_mode/status`、`/gps_guard/status`、`/neupan/scan_health`、`/neupan/route_connection`、`/neupan/camera_path_guard` 分别解释控制来源、定位、扫描、掉头及视觉路径状态。

每次运行应能关联源码、外部算法依赖、模型、地图、标定和实际参数。问题复现、测试结果与验收进度只写入 [PROJECT_ISSUES_CN.md](PROJECT_ISSUES_CN.md)，不在本文维护第二套清单。

## 8. 整个neupan_ros包：逐程序职责与改动

### 8.1 历史对照范围与数量（2026-10-30快照）

对照作者[neupan_ros固定提交b9558f3](https://github.com/hanruihua/neupan_ros/tree/b9558f3fbce4bdab356008b50b9eb6272cebef7e)，不是只比较`neupan_core.py`。排除`.git`和`__pycache__`，原包26个文件，当前128个；同路径修改4个、相同21个、新增103个、移除旧入口路径1个。旧入口`src/neupan_node.py`已迁移为`scripts/neupan_node.py`并补充异常诊断，并非删除了节点。逐文件状态和哈希见[整个包清单](tracker_logs/review_20261028/package_scope_followup/package_inventory.json)。

| 范围 | 当前文件数 | 与原版的关系 | 是否属于实车求解/控制代码 |
| --- | ---: | --- | --- |
| `src/`、`scripts/` | 6 | 核心修改1个、增加4个辅助模块、迁移并改写1个入口 | 是，具体启用条件见下表 |
| `example/lxw_car/` | 48 | 全部为本车新增 | 只有启动选中的配置/模型和节点参与；其中39个是模型及训练记录 |
| 已移除的本车仿真目录 | 50（历史） | 原新增仿真与对照工具 | 2026-11-06已移除 |
| 已移除的作者LIMO示例 | 18（历史） | 原与上游相同 | 2026-11-06已移除 |
| 包根目录 | 6 | CMake/README/package.xml修改，另外3个相同 | 构建、依赖、说明及安装辅助 |

当前包共有22个Python程序、2个C++程序和3个Shell脚本，下面全部列出。模型、地图、世界、图片、URDF和配置另列，不能把128个文件都理解成新增控制逻辑。

### 8.2 实车直接使用的Python程序

下表路径相对于`src/neupan_ros/`。上节数量与上游统计是精简前审计基线；不是当前控制逻辑计数。

| 程序 | 当前职责 | 对运动的影响 |
| --- | --- | --- |
| [scripts/neupan_node.py](src/neupan_ros/scripts/neupan_node.py) | ROS入口与异常报告 | 不单独计算转向 |
| [src/neupan_core.py](src/neupan_ros/src/neupan_core.py) | 扫描/同刻TF适配、参考与任务装载、单次原生求解、首动作发布、停车原因和日志 | 原生action直接发布；模式/任务变化、输入失效及异常时零速，不改写转角或热启动 |
| [src/neupan_safety.py](src/neupan_ros/src/neupan_safety.py) | 有限值/形状/硬上限、精确输入年龄和停车原因；保留独立历史危险所需几何工具 | 普通主线不再执行完整预测硬否决；不做速度重定时或连续性改写 |
| [src/neupan_navigation.py](src/neupan_ros/src/neupan_navigation.py) | `MotionFeedback`区间运动统计 | 只读诊断，不影响速度/转向/停车；无选点hook和制动包络 |
| [src/neupan_diagnostics.py](src/neupan_ros/src/neupan_diagnostics.py) | 内部异常栈与输入/输出快照 | 异常先停车再写日志；撤掉旧候选拒绝异步采集层 |

`neupan_execution.py`及其独立线程已删除。完整launch不再提供`checked_execution/geometric_obstacle_selection/停车标定`实验参数；仅把旧开关设false不能替代本次代码精简。原生本车100/10选点预算、模型和权重不变。50 cm采样与`enable_adaptive_adjust=false`仍保留。

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

### 8.4 已移除的非实车对照

2026-11-06按用户要求移除本车Gazebo、作者LIMO示例、TEB/RVO及其专属测试、构建目标和视频。`neupan_ros`仅安装实车入口和核心模块，不再声明Gazebo依赖。此前按固定上游提交的文件数量仅为历史审查快照，不能视作当前目录清单。

### 8.6 不在neupan_ros目录、但直接影响本车的代码

| 程序 | 与NeuPAN的关系 |
| --- | --- |
| [osm_camera_path_refinement.py](src/osm_map_handler/src/osm_camera_path_refinement.py) | 产生相机建议参考。默认`segment_reference`：直路上约1–3 m内保持，确认持续越界后平滑修正；可靠单右边界可用，无双边界本身不停车。独立真实边界危险仍保留 |
| [osm_camera_path_guard.py](src/osm_map_handler/src/osm_camera_path_guard.py) | 消费端核对参考身份/心跳、同任务参考保持、显式失效、独立危险及恢复；区别普通建议参考与旧显式视觉执行合同 |
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

以下作者版本对比与文件数量为2026-10-28审计快照，不能当作当前工作树状态。11月9日另对`neupan/blocks/nrmp.py`和`pan.py`作定向推理接口修改：显式CLARABEL复用原CVXPY问题；保留训练CvxpyLayer、PAN/DUNE迭代、原目标/约束和首动作。未建立外部控制器或自动求解器回退。历史差异不能统称为本轮新增功能。历史完整哈希/状态见[算法程序清单](tracker_logs/review_20261028/package_scope_followup/algorithm_inventory.json)，本轮验证见问题文档第5节。

### 9.2 算法包内全部13个Python文件

表中路径相对`/home/nuc/lxw/NeuPAN/`。“本机未改”指相对`947ddb0`未改，不表示与作者版本一致。

| 程序 | 在算法中的含义 | 本机工作树改动 | 与作者固定版本的差异及运行影响 |
| --- | --- | --- | --- |
| [neupan/__init__.py](/home/nuc/lxw/NeuPAN/neupan/__init__.py) | 导出`neupan`类，供ROS封装import | 未改 | 与py38/main一致 |
| [neupan/neupan.py](/home/nuc/lxw/NeuPAN/neupan/neupan.py) | 公共入口：读YAML、建立robot/InitialPath/PAN、生成每轮参考和名义控制、调用优化、保留热启动、返回首步动作和预测；处理到达与原生stop | 未改 | 与py38/main的差异为`q_s`标量/向量说明等文档；不能由文档变化推导本机已支持向量权重 |
| [neupan/blocks/__init__.py](/home/nuc/lxw/NeuPAN/neupan/blocks/__init__.py) | 导出6个算法组件 | 未改 | 与py38/main一致 |
| [neupan/blocks/initial_path.py](/home/nuc/lxw/NeuPAN/neupan/blocks/initial_path.py) | 路点生成线段/Dubins/Reeds参考，确定路径进度/挡位/到达，按上一轮控制生成名义状态，生成本轮50步参考 | **已改** | 主路径改用弧长与`abs(ref_speed)*dt`推进，插值位置/最短航向角，处理不均匀/重复点；增加有限值、每步位移及航向连续性校验；旧圆线求交辅助增加数值容差，拒绝无前向交点却跳到远端终点。详见9.3 |
| [neupan/blocks/pan.py](/home/nuc/lxw/NeuPAN/neupan/blocks/pan.py) | PAN迭代协调器：将名义轨迹转成各时刻点流，交替调用DUNE和NRMP，按变化量提前结束 | **已改** | 11月9日仅透传显式推理配置并启用NRMP推理模式，调用`train()`仍恢复原梯度路径。原静态障碍补零和默认dict等属于历史基线差异，非新增预测器 |
| [neupan/blocks/dune.py](/home/nuc/lxw/NeuPAN/neupan/blocks/dune.py) | 用ObsPointNet将障碍点编码为μ/λ，估计目标距离、排序并输出最小距离；载入或训练DUNE模型 | 未改 | 本机保留可变默认dict、未显式拒绝robot=None；缺模型/训练交互选择退出时使用`sys.exit`，作者版本采用显式异常及参数校验。模型与编码公式未因此重写 |
| [neupan/blocks/obs_point_net.py](/home/nuc/lxw/NeuPAN/neupan/blocks/obs_point_net.py) | DUNE的MLP网络结构，将二维点映射为车体边数对应的非负潜在特征 | 未改 | 与py38/main一致；新增ROS辅助代码不等于新增网络层 |
| [neupan/blocks/nrmp.py](/home/nuc/lxw/NeuPAN/neupan/blocks/nrmp.py) | 构造可微凸优化层，结合参考跟踪、避障距离变量、车体约束和线性化运动学求解控制/状态序列 | **已改** | 11月9日缓存原CVXPY问题并增加显式CLARABEL推理，验证状态/有限值/硬约束及返回精度残差；异常仍抛出，无自动回退。训练仍用原CvxpyLayer。本机`q_s`仍为标量，未移植作者向量权重实现 |
| [neupan/blocks/dune_train.py](/home/nuc/lxw/NeuPAN/neupan/blocks/dune_train.py) | 离线生成车体距离训练数据、训练/验证网络、保存检查点及测试结果 | 未改 | 与作者主要是保存路径变量拼写和注释差异；不是新增训练方法，普通导航不运行训练循环 |
| [neupan/robot/__init__.py](/home/nuc/lxw/NeuPAN/neupan/robot/__init__.py) | 导出robot类 | 未改 | 与py38/main一致 |
| [neupan/robot/robot.py](/home/nuc/lxw/NeuPAN/neupan/robot/robot.py) | 车体顶点/半空间、阿克曼/差速/全向线性化、参考代价及速度/预测内变化率约束 | 未改 | 本机参考状态代价用标量`q_s`；作者增加向量权重分支。原生相邻控制的变化率约束已存在，ROS额外首步连续性层已撤出 |
| [neupan/configuration/__init__.py](/home/nuc/lxw/NeuPAN/neupan/configuration/__init__.py) | Torch设备、NumPy/Tensor转换、数值类型和计时开关 | 未改 | 本机转换固定float32；作者版本统一`tensor_dtype`并扩展`np_to_tensor`梯度参数。与NRMP输出转换共同构成版本差异 |
| [neupan/util/__init__.py](/home/nuc/lxw/NeuPAN/neupan/util/__init__.py) | 计时、角度归一化、坐标变换、多边形半空间、文件查找与抽点 | 未改 | 与py38一致；对main主要差在`tuple[...]`类型注解，为Python 3.8兼容差异，不是新避障方法 |

### 9.3 已修改initial_path.py究竟改变什么

以前根据平均路径间距跨点，或用圆与后续线段求交来取参考；现在每步沿路径弧长推进`|v_ref|×dt`，避免50 cm重采样与0.1 s预测混用时错误跨点。当前室外0.8 m/s、0.1 s的目标推进量为每步0.08 m，50步总参考弧长最多约4 m；终点截断、弯曲或限速时不能简单等同于固定前视直线距离。

它调整的是优化器的参考和名义输入，不直接生成最终底盘指令，也不替代OSM或局部障碍优化。`curve_style='reeds'`表示一种参考曲线生成方式；最终输出是PAN/NRMP优化后的状态与控制序列，不能理解为“每次仅输出一条不再优化的RS曲线”。

### 9.4 源文件未改，也可能改变实际算法行为

| 变化入口 | 实际改变了什么 | 没有据此改变的内容 |
| --- | --- | --- |
| 室外YAML及可选自适应profile | 车体几何、时间步、预测长度、速度、控制限值和目标权重 | 改数值不等于更换优化算法；主launch自适应默认关闭 |
| 选中的`.pth` | DUNE实际推理权重 | 载入既有模型不等于本轮训练了新模型 |

普通实车仅由原生NeuPAN预测、跟踪和避障，ROS不再重建普通控制序列或设置全预测硬否决；相机真实独立危险仍保留原处理。仿真独立实验不进入实车主线。

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
| `collision_threshold` | 原生`check_stop()`使用的最小学习距离阈值，当前配置0.001 m | 不是固定停车距离；普通主线不再叠加0.05 m全预测硬否决，真实净距仍须实测 |

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
| 预测内速度/转角与变化率 | **已有**运动学与预测序列相邻控制约束 | 直接采用原生动作，不叠加ROS首步改写 |
| 原生碰撞停车 | **已有**`min_distance < collision_threshold`；学习最小距离与预测避障由原生算法处理 | 保留原生stop/arrive；不额外全扫描否决普通候选 |
| 完整扫描全预测的一票否决 | 原ROS封装没有这套门槛 | 本轮撤出；不宣称原生避障构成独立紧急制动保证 |
| 求解后精确绑定输入时效与恢复 | 原ROS扫描回调/位姿主要查最新TF，没有本车测量快照和恢复分类 | 属于新增职责；原版没有对应误分类分支，也未因此解决输入时效 |
| 求解暂慢时的独立控制段派发 | 原版在同一循环求解并发布首步动作 | 本轮撤掉旧线程；mux指令超时保护仍在 |
| 相机粗略靠右、单右边界和缺失回退 | 原版没有本项目这条链路 | 本工程新增OSM参考修正；原版不能作为该功能已实现的证明 |
| GNSS恢复/失锁、手动接管、MCU协议 | 原ROS包不提供本车整套定位/底盘实现 | 其他功能包承担，不能归为重写了NeuPAN目标函数 |

原版没有某个新增检查，既不能表述为“原版已经修好了这个检查的问题”，也不能表述为“原版完全没有避障”。当前具体缺陷、修改优先级及可交给执行模型的方案，统一见[问题文档](PROJECT_ISSUES_CN.md)。
