# NeuPAN 室内外导航项目分析

> 本文档用于帮助理解 `/home/nuc/lxw/neupan_ws` 的整体结构、运行链路和主要函数职责。
>
> 项目基于 ROS1 Noetic。`src/` 是源码，`build/` 和 `devel/` 是 catkin 生成物，通常不应直接修改。

## 1. 项目总体结构

```mermaid
flowchart LR
    CAM[Astra 相机] --> VISION[道路视觉检测]
    GPS[GPS/NMEA] --> LOC[GPS/LIO 定位保护]
    IMU[Livox IMU] --> LIO[FAST-LIO]
    LIDAR[Livox MID360] --> LIO
    LIO --> SCAN[点云转 LaserScan]
    MAP[OSM/栅格地图] --> ASTAR[OSM 全局 A*]
    LOC --> ASTAR
    ASTAR --> PATH[/osm_global_path]
    SCAN --> NP[NeuPAN]
    PATH --> NP
    LOC --> NP
    NP --> CMD[/neupan_cmd_vel]
    CMD --> BASE[底盘串口驱动]
    VISION --> REFINE[路径修正节点]
    REFINE --> PATH2[/osm_global_path_refined]
```

系统可以分成六个层次：

1. **传感器层**：Astra、Livox、GPS、IMU。
2. **定位层**：FAST-LIO、GPS 投影、GPS/LIO 守门器、`map -> odom` 对齐。
3. **地图与全局规划层**：OSM 道路图、A*、Hybrid A*、栅格地图。
4. **局部规划与控制层**：NeuPAN 或室外直接路径跟踪器。
5. **底盘执行层**：串口底盘驱动。
6. **测试与可视化层**：RViz、日志、rosbag、道路视觉调试。

## 2. 关键运行链路

### 2.1 室内 NeuPAN

```text
Livox -> FAST-LIO -> /cloud_registered_body
      -> pointcloud_to_laserscan -> /scan

/odom 或 AMCL TF -> map -> base_link
/scan + TF + /initial_path
      -> neupan_core.py
      -> /neupan_cmd_vel
      -> base_driver_node.py
      -> MCU 串口
```

### 2.2 室外 NeuPAN

```text
GPS + FAST-LIO
      -> /odometry/gps、/odometry/local
      -> global_odom_bridge.py
      -> /odometry/global
      -> map_odom_align_bridge.py
      -> map -> odom TF

GPS/RViz 目标 + OSM
      -> global_astar_node.cpp
      -> /osm_global_path
      -> neupan_core.py
      -> /neupan_cmd_vel
```

### 2.3 室外直接跟踪

```text
OSM A* -> /osm_global_path
       -> global_path_tracker.py
       -> /neupan_cmd_vel
       -> base_driver_node.py
```

`global_path_tracker.py` 和 `neupan_core.py` 都可能发布 `/neupan_cmd_vel`，不能在同一运行模式中同时启用。

### 2.4 道路视觉修正

```text
Astra RGB
  -> road_vision_node.py
  -> /road_vision/lateral_error
  -> path_refine_node.py
  -> /osm_global_path_refined
```

当前视觉链主要用于检测和验证。根据测试指南，完成静止检测和米制误差验收前，不应让 NeuPAN 使用 `/osm_global_path_refined`。

## 3. 根目录文件夹

### `src/`

所有 ROS 包和第三方源码的集合，是项目源码的主要入口。

### `build/`

catkin 编译目录，包含 CMake 缓存、构建规则、中间文件和生成的包构建目录。不要在这里修改源码。

### `devel/`

catkin 开发环境，包含 `setup.bash`、环境变量、Python 路径、动态库和 ROS 包索引。

### `bags/`

rosbag 数据目录。目前包含 `bags/road_test/`，用于道路视觉模式录制和离线复测。

### `tracker_logs/`

系统运行 JSONL 日志，记录 GPS、定位、路径、控制和安全状态。

### 根目录 Markdown

- `README.md`：项目总体说明。
- `README_SUMMARY.md`：项目摘要。
- `GDUT室外导航.md`：室外导航部署记录和说明。

## 4. `src/neupan_ros`

NeuPAN 的 ROS1 封装和示例工程。

### `src/neupan_ros/src/neupan_core.py`

核心类是 `neupan_core`。

- `__init__`：初始化 ROS 节点，读取规划器、LaserScan、TF、自车过滤和安全参数；加载 NeuPAN 配置；建立发布器和订阅器。
- `run`：50 Hz 主循环。读取 `map -> base_link` TF，检查路径和扫描健康状态，调用 NeuPAN，发布速度、路径、参考状态和 RViz Marker。
- `parse_adjust_profile`：把字符串形式的五元调参配置解析成 NumPy 数组。
- `mark_scan_valid`：记录扫描恢复时间和当前有效模式。
- `mark_scan_fault`：记录扫描故障并进入 `FAULT` 状态。
- `scan_is_healthy`：检查扫描是否存在、是否超时、TF 是否有效、是否已经稳定恢复。
- `publish_scan_health`：以 JSON 字符串发布 `/neupan/scan_health`。
- `compute_min_obstacle_distance`：计算机器人前方指定角度范围内的最近障碍距离。
- `update_adaptive_adjust_parameters`：按最近障碍距离，在 clear 和 avoid 两组 NeuPAN 参数之间插值。
- `scan_callback`：过滤无效距离、按角度和距离筛选、降采样，并把雷达点从 `lidar_frame` 转换到地图坐标。
- `path_callback`：接收完整 `nav_msgs/Path`，转换到 `map` 后交给规划器。
- `waypoints_callback`：从机器人当前位置和多个航点生成规划初始路径。
- `goal_callback`：从当前位姿和目标 Pose 生成规划初始路径。
- `quat_to_yaw_list`：将四元数数组转换为偏航角。
- `quat_to_yaw`：将 ROS Quaternion 转换为偏航角。
- `yaw_to_quat`：将偏航角转换为 ROS Quaternion。
- `generate_path_msg`：把 NeuPAN 内部路径转换成 `nav_msgs/Path`。
- `generate_twist_msg`：把 NeuPAN 动作转换成 `geometry_msgs/Twist`；到达或停止时输出零速度。
- `generate_dune_points_markers_msg`：发布 DUNE 点集 MarkerArray。
- `generate_nrmp_points_markers_msg`：发布 NRMP 点集 MarkerArray。
- `generate_robot_marker_msg`：根据机器人尺寸和运动学生成 RViz 车体 Marker。

主要输入话题：`/scan`、`/initial_path`、`/neupan_waypoints`、`/neupan_goal`。

主要输出话题：`/neupan_cmd_vel`、`/neupan_plan`、`/neupan_initial_path`、`/neupan_ref_state`、`/dune_point_markers`、`/nrmp_point_markers`、`/robot_marker`、`/neupan/scan_health`。

### `src/neupan_ros/src/neupan_node.py`

程序入口。实例化 `neupan_core`，然后调用 `run()`。

### `example/lxw_car`

真实阿克曼车辆示例。

- `config/neupan_config.yaml`：旧版默认配置入口。
- `config/neupan_config_indoor.yaml`：室内 NeuPAN 参数。
- `config/neupan_config_outdoor.yaml`：室外较保守参数。
- `config/neupan_train.yaml`：DUNE 训练参数。
- `dune_train_lxw_car.py`：加载训练配置并调用 `train_dune()`。
- `launch/neupan_lxwcar.launch`：真实车辆基础启动，包含底盘、FAST-LIO、Livox、点云转 LaserScan 和 NeuPAN。
- `launch/full_system_run.launch`：完整室内/室外系统入口，包含定位、GPS、OSM A*、点云处理、视觉相关参数和跟踪器。
- `launch/outdoor_global_only_test.launch`：只测试室外定位和 OSM 全局规划。

### `example/lxw_car_simulation`

阿克曼车辆 Gazebo 仿真。

- `config/neupan_config_sim.yaml`：仿真规划参数。
- `scripts/ackermann_sim_driver.py`：接收速度指令，按阿克曼模型更新 Gazebo 车辆状态，发布里程计和 TF。
- `scripts/publish_demo_goal.py`：延时发布仿真目标。
- `launch/lxw_car_simulation.launch`：仿真总入口。
- `launch/neupan_lxwcar_gazebo.launch`：启动 Gazebo、车辆模型、RVO 动态障碍、仿真驱动、NeuPAN 和 RViz。
- `worlds/`、`models/`、`urdf/`、`rviz/`：仿真环境、车辆模型、机器人描述和显示配置。

### `example/gazebo_limo`

Limo 差速车仿真。

- `config/neupan_planner_limo.yaml`：循环路径配置。
- `config/neupan_planner_limo_noloop.yaml`：非循环路径配置。
- `dune_train_limo.yaml`：Limo DUNE 训练参数。
- `dune_train_limo.py`：Limo DUNE 训练入口。
- `launch/neupan_gazebo_limo.launch`：启动 NeuPAN 和 RViz。
- `launch/neupan_gazebo_limo_noloop.launch`：非循环版本。
- `launch/gazebo_limo_env_complex_20.launch`：启动复杂 Gazebo 环境和 RVO 动态障碍。
- `launch/run_limo_rviz.launch`：只启动模型状态发布和 RViz。

## 5. `src/robot_base_driver`

负责底盘串口、里程计、定位保护、路径跟踪和系统日志。

### `src/base_driver_node.py`

- `RobotBaseDriver.__init__`：读取串口、车体和安全参数，打开串口，建立速度订阅、里程计发布和 TF 广播。
- `cmd_vel_callback`：接收 `/neupan_cmd_vel`，把线速度和转向角转换为 MCU 协议 `CMD,<throttle>,<steering>`；处理死区和转角限幅。
- `localization_safety_callback`：定位不安全时立即发送停车命令。
- `send_serial_command`：格式化并写入串口。
- `read_from_mcu`：读取 MCU 的 `ODO` 行，解析位置和航向，设置里程计零点，计算相对位姿和速度。
- 主循环：持续读取 MCU 数据并发布 `/odom`，必要时发布 `odom -> base_link` TF。

### `src/global_path_tracker.py`

室外不经过 NeuPAN 的直接路径跟踪器。

- `clamp`：限制数值范围。
- `wrap_to_pi`：把角度归一化到 $[-\pi, \pi]$。
- `GlobalPathTracker.__init__`：读取路径、里程计、目标、前视、速度、转角和停车参数。
- `path_callback`：缓存 `/osm_global_path`，计算路径累计长度并重置目标状态。
- `odom_callback`：接收全局里程计并提取位姿。
- `update_tf_pose`：当启用 TF 模式时读取当前机器人位姿。
- `goal_callback`：记录 RViz 点击目标。
- `publish_stop`：发布零速度。
- `_record_debug_sample`：发布 JSON 调试状态。
- `compute_cumulative_lengths`：计算路径每个点的累计弧长。
- `project_onto_path`：将机器人投影到最近路径段，得到横向误差、弧长和剩余距离。
- `segment_heading_at_projection`：计算投影位置处的路径切线方向。
- `find_target_point`：根据前视距离找到跟踪目标点。
- `should_stop_at_goal`：结合点击目标、路径末端和越界条件判断是否停车。
- 控制循环中的控制计算：根据横向误差和航向误差计算 Ackermann 转角，根据距离目标的远近调整速度。

### `src/gps_fix_to_map_odom.py`

- `latlon_to_xy`：把经纬度转换到以 datum 为原点的局部平面坐标。
- `fix_cb`：缓存 `NavSatFix`。
- `heading_cb`：缓存航向。
- `build_covariance`：从 GPS 协方差构造里程计协方差。
- `timer_cb`：周期发布 `/odometry/gps`。

### `src/global_odom_bridge.py`

GPS 与 FAST-LIO 的全局定位安全守门器。

- GPS、LIO、GGA、航向回调：缓存各类定位和质量数据。
- `data_quality`：检查 fix 状态、GGA 质量、卫星数、HDOP 和位置协方差。
- `candidate_transform`：根据 GPS 与局部里程计计算候选 `map -> odom` 变换。
- `transform_error`：计算候选变换与当前变换的误差。
- `candidates_consistent`：检查连续候选是否稳定一致。
- `append_recovery_candidate`：加入 GPS 恢复候选窗口。
- `recovery_window_stable`：判断恢复窗口是否满足稳定时间。
- `set_mode`：切换 GPS 跟踪、GPS 丢失、本地续推和恢复状态。
- `update_guard`：发布 `/odometry/global`、`/gps_guard/status` 和 `/localization_safe`。

### `src/lio_odom_guard.py`

FAST-LIO 输出保护器。

- `pose_from_odom`：提取里程计位置和偏航角。
- `assess_motion`：检查平面速度、横向速度、角速度和采样间隔。
- `apply_offset`：对 LIO 位姿施加当前对齐偏移。
- `align_offset`：重新计算 LIO 与轮速之间的偏移。
- `wheel_extrapolated_target`：根据轮速里程计外推目标位姿。
- `publish_corrected`：发布保护后的 LIO 里程计。
- `lio_cb`、`wheel_cb`：处理 FAST-LIO 和轮速输入。

### `src/map_odom_align_bridge.py`

- `_global_cb`：缓存全局里程计。
- `_local_cb`：缓存局部里程计。
- `_timer_cb`：根据全局和局部里程计实时计算并广播 `map -> odom`。

### `src/system_run_logger.py`

- `topic_map`：定义需要记录的话题集合。
- `open_log_file`：创建 JSONL 日志文件。
- `write_record`：写入单条记录。
- 各类 `*_cb`：缓存 GPS、GGA、航向、里程计、控制、路径和安全状态。
- `timer_cb`：按频率把综合状态写入日志。

### `config/` 和 `launch/`

包含底盘串口、EKF、GPS、LIO 守门、导航核心和 TF 启动配置。启动文件决定哪些节点同时运行，是排查 TF 冲突和重复控制器的首要位置。

## 6. `src/osm_map_handler`

负责 OSM 道路图加载、道路吸附、A* 全局路径和 GPS 目标转换。

### `src/global_astar_node.cpp`

核心类通常为 `OSMGraphAStarPlanner`。

- `loadGraph`：读取 OSM XML，过滤道路并建立节点、边和道路属性。
- `latLonToXY`：经纬度转换到局部地图坐标。
- `projectPointToSegment`：将点投影到道路线段。
- `findNearestGraphPoint`：寻找最近道路图位置，用于起点/终点吸附。
- `buildGraphPath`：在道路图上执行 A* 搜索。
- `applyRightLaneOffset`：根据道路宽度、行驶方向和车道参数把中心线偏移到右侧车道。
- `computeTurnAngle`：计算连续路径段的转角。
- `spacingFromTurnAngle`：根据转角决定路径采样间距。
- `resamplePathForTracking`：生成适合车辆控制的稠密路径。
- `prepareGoalRequest`：准备当前起点和目标请求。
- `processGoal`：执行吸附、搜索、偏移、重采样并发布最终路径。
- `mapCallback`：接收地图或地图数据。
- `odomCallback`：接收机器人位姿。
- `gpsCallback`：接收 GPS。
- `goalCallback`：处理 RViz 或外部目标。
- `clickedPointCallback`：处理 RViz 点击点。
- `timerCallback`：维护 GPS Marker 和目标超时状态。

主要输出：`/osm_global_path`、`/osm_vector_map`、`/hybrid_astar_subgoal`、`/initialpose`。

### `scripts/gps_goal_bridge.py`

把经纬度目标转换为局部 `PoseStamped`，发布到 `/move_base_simple/goal`。

### `scripts/osm_alignment_analyzer.py`

离线分析 GPS 轨迹与 OSM 道路的对齐误差。

- `load_osm_segments`：加载道路线段。
- `load_points`：读取 GPS 或测试点。
- `nearest_segment`：寻找每个点的最近道路段。

## 7. `src/road_vision_refinement`

负责相机道路区域、中心线、道路右边缘和路径横向修正。

### `scripts/road_vision_node.py`

核心类为 `RoadVisionNode`。

- `__init__`：读取 HSV、ROI、IPM、置信度、平滑、确认帧数和输出参数，创建 OpenCV 核和 ROS 接口。
- `load_ipm_matrix`：读取图像到 `base_link` 地面坐标的 3x3 单应矩阵，并检查矩阵合法性。
- `image_cb`：图像处理主入口，执行尺寸检查、ROI 裁剪、道路 mask、中心线检测、来源选择和图像/状态发布。
- `make_road_mask`：用低饱和度 HSV 阈值提取道路区域，并进行开闭运算。
- `make_center_line_mask`：用白色和黄色 HSV 阈值提取中心线候选。
- `detect_center_line`：使用 HoughLinesP、长度、倾角、历史位置和方向条件筛选中心线，并计算置信度。
- `center_line_target`：在确认的中心线右侧生成目标。
- `estimate_road_region`：寻找最大道路候选轮廓，估计扫描线范围、道路置信度和底部接触比例。
- `road_edge_target`：把道路候选右边缘作为目标来源。
- `road_region_target`：把道路区域中心作为目标来源。
- `project_image_to_ground`：使用 IPM 把像素目标投影到地面坐标。
- `publish_selected`：发布有效目标、误差、来源和调试状态。
- `publish_invalid`：目标失效时执行短暂保持或无视觉参考状态。
- `publish_metric_unavailable`：IPM 未加载或图像尺寸不匹配时发布不可度量状态。
- `publish_status`：发布 JSON 状态。
- `draw_center_line`、`draw_road_region`：在调试图上绘制检测结果。
- `publish_images`：发布 mask、中心线 mask、道路 mask 和 debug image。

自动模式优先级：

```text
center_line -> road_edge -> hold_last_valid -> no_visual_reference
```

### `scripts/path_refine_node.py`

- `path_cb`：缓存 `/osm_global_path`。
- `error_cb`：缓存视觉横向误差。
- `status_cb`：缓存视觉来源和有效状态。
- `publish_refined`：按误差修正路径横向位置，进行保持、衰减和平滑，然后发布 `/osm_global_path_refined`。

### 标定和采图脚本

- `save_one_image.py`：等待下一帧 ROS 图像，保存后退出。
- `pick_ipm_points.py`：在图像上人工点击已知地面点，计算 IPM 矩阵并保存 JSON。
- `verify_ipm_points.py`：使用未参与标定的新点验证预测坐标和误差。

### `config/`

保存 IPM 点、相机内参、视觉阈值和道路检测参数。

### `launch/`

包含相机、`image_proc`、视觉节点、调试图像查看器和路径修正的启动组合。

## 8. `src/hybrid-a-star-annotation`

带车辆运动学约束的 Hybrid A* 路径规划器。

主要调用关系：

```text
Planner::plan
  -> Algorithm::hybridAStar
  -> Node3D / Node2D
  -> CollisionDetection
  -> Smoother
  -> Path / RViz 发布
```

- `planner.cpp`：ROS 规划器总控。
  - `initializeLookups`：初始化 Dubins 和碰撞查找表。
  - `setMap`、`setStart`、`setGoal`：更新地图、起点和终点。
  - `plan`：执行搜索、路径回溯、平滑和发布。
- `algorithm.cpp`：核心搜索。
  - `hybridAStar`：三维状态空间搜索，状态包含位置和朝向。
  - `aStar`：二维栅格搜索，用于启发式代价。
  - `updateH`：融合二维 A* 与 Dubins/Reeds-Shepp 启发式。
  - `dubinsShot`：尝试直接用曲线连接当前节点和目标。
- `node2d.cpp/.h`：二维节点的索引、代价、开放/关闭状态和邻居生成。
- `node3d.cpp/.h`：带方向角的车辆节点和运动学后继生成。
- `collisiondetection.cpp/.h`：车辆轮廓栅格化和碰撞检查。
- `smoother.cpp/.h`：路径回溯、障碍代价、曲率代价和平滑度代价。
- `path.cpp/.h`：路径段、节点和车辆 Marker 的生成。
- `visualize.cpp/.h`：搜索节点、代价和热力图可视化。
- `dynamicvoronoi.cpp/.h`：动态 Voronoi 距离场。
- `dubins.cpp/.h`：Dubins 曲线长度、采样和子路径计算。
- `tf_broadcaster.cpp`：发布规划相关 TF。
- `main.cpp`：ROS 节点入口。

## 9. `src/FAST_LIO`

Livox 点云与 IMU 融合定位系统。

### `src/laserMapping.cpp`

- `main`：读取参数，初始化 ROS 发布器、订阅器、预处理器和 EKF。
- `standard_pcl_cbk`：接收标准 PointCloud2。
- `livox_pcl_cbk`：接收 Livox 点云。
- `imu_cbk`：缓存 IMU 数据。
- `sync_packages`：同步 IMU 和点云时间段。
- `h_share_model`：构造点到地图平面的 EKF 残差和雅可比。
- `map_incremental`：把当前有效点加入 ikd-Tree 地图。
- `lasermap_fov_segment`：维护当前视野内的局部地图。
- `pointBodyToWorld`：机体坐标点转换到世界坐标。
- `publish_frame_world`、`publish_frame_body`：发布世界/机体坐标点云。
- `publish_effect_world`：发布参与匹配的有效点。
- `publish_map`：发布地图点云。
- `publish_odometry`：发布 `/Odometry`。
- `publish_path`：发布 FAST-LIO 轨迹。

### `src/preprocess.cpp`

- `process(CustomMsg)`：处理 Livox 自定义消息。
- `process(PointCloud2)`：处理标准 ROS 点云。
- `avia_handler`、`oust64_handler`、`velodyne_handler`、`sim_handler`：不同雷达类型的数据解析。
- `give_feature`：提取平面和边缘特征。
- `plane_judge`：平面特征判断。
- `edge_jump_judge`：边缘跳变判断。

### `src/IMU_Processing.hpp`

类 `ImuProcess`：

- `Reset`：重置初始化状态。
- `set_extrinsic`：设置 LiDAR 与 IMU 外参。
- 协方差 setter：设置陀螺仪、加速度计和过程噪声。
- `IMU_init`：估计重力方向和 IMU 偏置。
- `UndistortPcl`：使用 IMU 运动补偿点云畸变。
- `Process`：完成 IMU 初始化、预积分和点云去畸变。

### `include/`

- `ikd-Tree/`：动态 KD-Tree，负责地图近邻搜索、插入、删除和重建。
- `IKFoM_toolkit/`：流形上的迭代误差状态 EKF 工具。
- `matplotlibcpp.h`：第三方 matplotlib C++ 封装。
- `common_lib.h`、`use-ikfom.hpp`、`so3_math.h`：状态、旋转和公共数学工具。

## 10. `src/livox_ros_driver2` 与 `src/Livox-SDK2`

这两部分属于 Livox 底层驱动和官方 SDK，主要用于设备通信，不是导航策略代码。

### `livox_ros_driver2`

- `livox_ros_driver2.cpp`：ROS1/ROS2 入口，读取设备和输出格式参数。
- `driver_node.cpp/.h`：管理 LDDC、LDS、雷达线程和 ROS 发布。
- `lds.cpp/.h`：雷达数据缓存、队列和退出控制。
- `lds_lidar.cpp/.h`：雷达初始化、配置、回调绑定和释放。
- `lddc.cpp/.h`：从 SDK 队列读取数据并发布 PointCloud2、CustomMsg、PCL 和 IMU。
- `comm/`：队列、信号量、缓存、数据解包和拼帧。
- `parse_cfg_file/`：JSON 配置读取。
- `call_back/`：Livox SDK 数据和设备状态回调。
- `msg/`：Livox 自定义 ROS 消息。
- `launch_ROS1/`、`launch_ROS2/`：不同 ROS 版本启动文件。
- `3rdparty/rapidjson/`：第三方 JSON 库。

### `Livox-SDK2`

官方 SDK：

- `sdk_core/`：设备发现、网络通信、协议解析、日志和设备控制。
- `include/`：公开 API 和协议结构。
- `samples/`：官方示例。
- `3rdparty/`：SDK 依赖。
- `build/`：SDK 编译产物。

排查雷达连接、设备发现、协议或丢包问题时才需要深入阅读这里。

## 11. `src/rvo_ros`

Gazebo 动态障碍物和 RVO 避碰仿真。

- `src/rvo_node.cpp`：ROS 节点入口，读取 Gazebo 模型状态、运行 RVO 并提供目标服务。
- `rvo_lib/nav_rvo.cpp`：RVO ROS 封装。
  - `setupScenario`：设置邻居距离、时间视界、半径和最大速度。
  - `setGoal`：设置固定目标。
  - `randomOnceGoal`、`randGoal`：生成随机目标。
  - `arrived`：判断 agent 是否到达目标。
  - `setInitial`：判断 agent 是否初始化完成。
  - `setPreferredVelocities`：计算朝向目标的期望速度。
  - `updateState_gazebo`：将 Gazebo ModelStates 转成 RVO 状态。
  - `step`：执行邻居搜索和新速度求解。
- `Agent.cpp/.h`：单个 agent 的 ORCA 约束和速度求解。
- `RVOSimulator.cpp/.h`：RVO 仿真器和时间步进。
- `KdTree.cpp/.h`：空间索引和邻居查询。
- `Obstacle.cpp/.h`：障碍物表示。
- `Vector2.h`：二维向量运算。
- `formation.cpp`：队形辅助。
- `set_goals_client.cpp`：调用设置目标服务。
- `srv/SetGoals.srv`：设置动态障碍物目标。
- `simulation/`：Gazebo 世界、插件和资源。

## 12. `src/pcd2pgm_package`

将 PCD 点云转换为二维栅格地图。

主要文件 `pcd2pgm/pcd2pgm.cpp`：

- `PassThroughFilter`：按高度范围过滤点。
- `RadiusOutlierFilter`：剔除稀疏离群点。
- `SetMapTopicMsg`：根据点云范围和分辨率生成 `nav_msgs/OccupancyGrid`。
- `main`：读取参数、执行点云滤波并发布地图。

## 13. `src/teb_test`

把 OSM 全局路径接入 `move_base`/TEB。

### `src/osm_path_planner.cpp`

核心类 `OSMPathPlanner : nav_core::BaseGlobalPlanner`：

- `initialize`：读取路径话题、坐标系、超时和匹配容差。
- `makePlan`：等待与当前 `move_base` 请求匹配的 `/osm_global_path`，返回计划。
- `pathCallback`：缓存 OSM 路径。
- `isUsablePlan`：检查路径时间戳、起点误差和终点误差。

`plugins.xml` 负责向 `move_base` 注册该全局规划插件，`config/` 和 `launch/` 保存 TEB、costmap 和 move_base 参数。

## 14. `src/wheeltec_gps_driver`

NMEA GPS 驱动和 GPS 辅助工具。

### `src/libnmea_navsat_driver/parser.py`

- `safe_float`、`safe_int`：安全数值转换。
- `convert_latitude`、`convert_longitude`：NMEA 度分格式转十进制度。
- `convert_time`、`convert_time_rmc`：解析 NMEA 时间。
- `convert_status_flag`：解析定位状态。
- `convert_knots_to_mps`：节转换为米每秒。

### 其他文件

- `checksum_utils.py`：校验 NMEA checksum。
- `driver.py`：类 `RosNMEADriver`，将 NMEA 解析结果转换成 ROS 消息并发布。
- `nmea_serial_driver.py`：串口读取 NMEA。
- `nmea_socket_driver.py`：UDP 接收 NMEA。
- `nmea_topic_driver.py`：订阅原始 NMEA 话题并解析。
- `nmea_topic_serial_reader.py`：串口读取并发布原始 NMEA 话题。
- `gps_path.cpp`：接收 GPS、累计 GPS 路径并发布相关 TF。
- `test/test_driver.py`：使用 rosbag 检查 GPS 驱动输出。

其中 `libnmea_navsat_driver` 基于通用 NMEA 驱动改动，阅读时应区分通用代码和本地修改。

## 15. `src/Test`

车辆开环测试和数据记录。

### `scripts/open_loop_motion.py`

- `speed_scale`：调整测试速度。
- `steering_at`：设置不同阶段的转角。
- `publish_stop`：发布停车指令。
- `wait_for_required_sources`：等待 GPS、定位等必要输入。
- 主循环：向 `/neupan_cmd_vel` 发布开环控制并发布运动状态。

### `scripts/open_loop_data_logger.py`

- `build_packet`：构造 GPS、里程计、控制量和状态的综合记录。
- `packet_sources_ready`：检查数据源是否完整。
- `write_record`：写入 JSONL 文件。
- 各类回调：缓存 GPS、航向、三类里程计、控制和运动状态。

## 16. `src/ros_astra_camera-main`

Astra/Orbbec 相机驱动，属于传感器基础设施。

主要职责：

- 发现和管理相机设备。
- 发布 RGB、深度和点云。
- 发布 CameraInfo。
- 提供 OpenNI、UVC、服务和动态配置接口。
- 提供相机内部 TF。

道路视觉主要使用校正后的 `/camera/rgb/image_rect_color` 和 `/camera/rgb/camera_info`。

## 17. 其他源码与资源包

### `src/nmea_msgs-master`

ROS NMEA 消息定义包，主要提供消息类型。

### `src/urdf`

机器人模型和传感器安装结构，影响机器人外形、关节和 TF。

### `src/FAST_LIO/Log`、`PCD`、`rviz_cfg`

FAST-LIO 运行日志、点云数据和 RViz 配置。

### 各包的 `CMakeLists.txt` 和 `package.xml`

- `package.xml`：定义 ROS 包名、依赖和运行时依赖。
- `CMakeLists.txt`：定义消息生成、编译目标、链接库、脚本安装和测试。
- launch 文件：决定节点、参数、话题 remap、TF 和启动条件。

## 18. 重点配置概念

### NeuPAN 配置

- `receding`：预测步数。
- `step_time`：模型时间步长。
- `ref_speed`：参考速度。
- `robot.kinematics`：`acker` 或 `diff`。
- `robot.wheelbase`：阿克曼轴距。
- `robot.max_speed`、`robot.max_acce`：速度和动作变化限制。
- `ipath`：初始路径间隔、曲线类型、循环和到达阈值。
- `pan`：DUNE/NRMP 迭代和模型参数。
- `adjust`：路径跟踪和避障权重。

### 定位安全配置

主要位于 `full_system_run.launch`：

- GPS fix 状态、GGA quality、卫星数、HDOP 和位置协方差门限。
- GPS 航向跳变和位置跳变门限。
- GPS 丢失后的本地续推时间。
- GPS 恢复所需稳定时间。
- FAST-LIO 速度、横向速度、角速度和采样间隔门限。

### 道路视觉配置

- ROI 顶部比例。
- 白色/黄色中心线 HSV 阈值。
- 道路低饱和度 HSV 阈值。
- 中心线最小长度、最大位置跳变和最大方向跳变。
- 中心线连续确认帧数。
- 道路轮廓最少扫描线数和底部接触比例。
- IPM 矩阵文件和图像尺寸。
- 无效保持时间和衰减系数。

## 19. 当前项目的主要风险

1. **速度话题争抢**：NeuPAN 和 `global_path_tracker.py` 不应同时发布 `/neupan_cmd_vel`。
2. **TF 冲突**：多个 launch 可能同时发布 `map -> odom`、`odom -> base_link` 或传感器外参。
3. **传感器重复启动**：Astra、Livox、FAST-LIO 和 `image_proc` 若由多个 launch 启动，会出现重复节点或重复话题。
4. **GPS 恢复跳变**：GPS 从失锁恢复时必须经过质量、连续稳定和位置/航向跳变检查。
5. **LaserScan 安全停机**：扫描缺失、TF 不可用或数据长期被过滤时，NeuPAN 应保持停车。
6. **视觉误检**：墙体、阴影、地砖缝和反光区域可能被识别为道路或中心线。
7. **视觉坐标方向**：相机图像右侧不一定等于 `base_link` 的右侧，当前逻辑需要结合安装方向和实测验证。
8. **IPM 误差**：当前独立验证误差约 `0.225 m`，暂按 `0.25 m` 验收，实车还需继续记录系统偏差。
9. **全局路径修正范围**：整条路径平移可能导致远处路径不合理，后续应改为车辆前方局部修正。

## 20. 推荐阅读顺序

1. `src/neupan_ros/example/lxw_car/launch/full_system_run.launch`
2. `src/neupan_ros/src/neupan_core.py`
3. `src/robot_base_driver/src/base_driver_node.py`
4. `src/robot_base_driver/src/global_odom_bridge.py`
5. `src/robot_base_driver/src/global_path_tracker.py`
6. `src/osm_map_handler/src/global_astar_node.cpp`
7. `src/FAST_LIO/src/laserMapping.cpp`
8. `src/road_vision_refinement/scripts/road_vision_node.py`
9. 各包的 `config/`、`launch/`、`package.xml` 和 `CMakeLists.txt`
10. Livox SDK、Astra 驱动、`build/` 和 `devel/`

## 21. 推荐调试方法

### 查看节点和话题

```bash
rosnode list
rostopic list
rostopic info /neupan_cmd_vel
rostopic info /scan
```

### 查看 TF

```bash
rosrun tf view_frames
rosrun tf tf_echo map base_link
rosrun tf tf_echo map odom
```

### 查看关键状态

```bash
rostopic echo /neupan/scan_health
rostopic echo /gps_guard/status
rostopic echo /localization_safe
rostopic echo /road_vision/status
```

### 道路视觉测试

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch road_vision_refinement road_vision_debug.launch
```

车辆保持静止，不要同时启动完整导航或接入 NeuPAN。先验证：

```text
中心线 -> 道路右边缘 -> 暂时失效保持 -> 持续失效
```

### 录包复测

重点记录：

```text
/camera/rgb/image_rect_color
/camera/rgb/camera_info
/road_vision/debug_image
/road_vision/center_line_mask
/road_vision/road_mask
/road_vision/status
/road_vision/lateral_error
/osm_global_path
/osm_global_path_refined
/scan
/tf
/tf_static
```

## 22. 一句话总结

这是一个将 GPS、FAST-LIO、Livox、Astra、OSM、Hybrid A*、NeuPAN 和真实底盘串口整合到 ROS1 中的室内外导航系统：OSM 负责全局路径，FAST-LIO/GPS 负责定位，LaserScan 负责局部障碍，NeuPAN 或直接跟踪器负责控制，底盘驱动负责最终执行，道路视觉则作为独立的道路偏移检测和路径修正模块。
