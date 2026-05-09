# 项目代码含义说明

生成时间: 2026-05-09

本文档基于当前代码内容整理, 以包和文件为单位说明主要用途与函数/类职责. 已覆盖: neupan_ros, robot_base_driver, osm_map_handler. 其余包将在后续追加.

---

## neupan_ros

### src/neupan_core.py

**类 `neupan_core`**

- `__init__`: 初始化 ROS 节点与参数, 载入 NeuPAN 规划器, 创建发布/订阅话题与 TF 监听器.
- `run`: 主循环. 从 TF 获取 `map->base` 位姿, 构造 `robot_state`, 若初始路径就绪则调用规划器输出速度与路径, 同时发布可视化 Marker.
- `scan_callback`: 将 `/scan` 的 `LaserScan` 角度与距离投影到 `map` 坐标系, 得到 `obstacle_points`.
- `path_callback`: 接收 `/initial_path` 的 `Path`, 统一到 `map` 坐标系, 生成 `(x,y,theta)` 初始路径, 设置规划器初始路径.
- `waypoints_callback`: 接收 `/neupan_waypoints` 路径, 用点列构建初始路径并刷新.
- `goal_callback`: 接收 `/neupan_goal`, 由当前状态+目标位置生成初始路径.
- `quat_to_yaw_list`: 将四元数数组转为偏航角.
- `generate_path_msg`: 将规划器内部路径列表转 `nav_msgs/Path`.
- `generate_twist_msg`: 将规划器控制量转 `Twist`, 在 `stop/arrive` 状态下输出零速度.
- `generate_dune_points_markers_msg`: 发布 DUNE 采样点 Marker.
- `generate_nrmp_points_markers_msg`: 发布 NRMP 采样点 Marker.
- `generate_robot_marker_msg`: 用矩形 Marker 显示车体外形与姿态.
- `yaw_to_quat`: 偏航角转四元数.
- `quat_to_yaw`: 四元数转偏航角.

**主要话题**

- 订阅: `/scan`, `/initial_path`, `/neupan_waypoints`, `/neupan_goal`
- 发布: `/neupan_cmd_vel`, `/neupan_plan`, `/neupan_ref_state`, `/neupan_initial_path`, `/dune_point_markers`, `/nrmp_point_markers`, `/robot_marker`

### src/neupan_node.py

- 入口脚本. 创建 `neupan_core` 并调用 `run()`.

### example/lxw_car/dune_train_lxw_car.py

- 使用训练配置 `config/neupan_train.yaml` 初始化 NeuPAN, 调用 `train_dune()` 训练模型.

### example/lxw_car/scripts/outdoor_runtime_diagnose.py

- 室外运行诊断工具. 轮询 `/gps/fix`, `/odometry/gps`, `/odometry/global`, `/odometry/local`, `/scan` 以及 OSM Marker, 输出 GPS 与全局融合一致性检查, 并检查 TF 链.

### example/lxw_car/scripts/outdoor_gps_goal_test.py

- GPS 目标测试工具. 向 `/gps_goal_ll` 发布 `NavSatFix` 并校验 `/move_base_simple/goal` 是否正确生成.

### source_setup.sh

- 将 `devel/setup.bash` 或 `devel/setup.zsh` 写入 shell 配置, 便于 ROS 环境自动加载.

### example/lxw_car/launch/neupan_lxwcar.launch

- 真车部署: 启动底盘驱动, FAST-LIO, Livox 驱动, 点云转 `LaserScan`, 静态 TF, NeuPAN 控制节点.
- 关键参数: `config_file`, `map_frame`, `lidar_frame`, `scan_range`, `scan_angle_range`, `dune_checkpoint`.

### example/lxw_car/launch/neupan_lxwcar_norviz.launch

- 与 `neupan_lxwcar.launch` 类似, 但默认关闭 RViz.

### example/lxw_car/launch/full_system_run.launch

- 全系统启动: 室内/室外模式参数, GPS/OSM 配置, EKF 组合, OSM 地图与矢量规划器, FAST-LIO 与点云转 2D, NeuPAN 节点.

### CMakeLists.txt / package.xml

- ROS 包定义与依赖: `rospy`, `roscpp`, `std_msgs`.

---

## robot_base_driver

### src/base_driver_node.py

**类 `RobotBaseDriver`**

- `__init__`: 读取串口与 GPS 滤波参数, 初始化串口, 订阅 `/neupan_cmd_vel`, 发布 `/odom` 和 `/gps/fix`, 创建 TF 广播器.
- `_parse_gps_lat_lon`: 解析 GPS 文本格式, 兼容 `GPS,lat,lon` 与 `GPS,lat,N,lon,E`.
- `_convert_dm_to_decimal`: 将 ddmm.mmmm 转十进制度数.
- `_haversine_distance_m`: 计算两经纬度的球面距离.
- `_smooth_lat_lon`: 滑动窗口均值滤波.
- `_ema_lat_lon`: 指数滑动平均平滑.
- `cmd_vel_callback`: 将速度指令转为下位机串口协议 `CMD,<throttle>,<steering>`; 含死区和转向限幅.
- `read_from_mcu`: 读取下位机数据.
  - `ODO` 数据: 解析位置与航向, 计算里程计, 可选发布 `odom->base` TF 与 `Odometry`.
  - `GPS` 数据: 解析纬经度, 进行跳变/静止漂移抑制, 发布 `NavSatFix`.
- `run`: 主循环读取串口并发布.

### src/global_odom_bridge.py

**类 `GlobalOdomBridge`**

- `__init__`: 订阅 `/odometry/gps`, 发布 `/odometry/global`.
- `_finite`: 检查数值是否有限.
- `cb`: 过滤非有限数据, 复用输入消息更新 frame 后发布.

### src/map_odom_align_bridge.py

**类 `MapOdomAlignBridge`**

- `__init__`: 订阅 `/odometry/global` 与 `/odometry/local`, 定期发布 `map->odom` TF.
- `_yaw_from_quat`: 从四元数提取偏航.
- `_global_cb` / `_local_cb`: 缓存最近消息与时间戳.
- `_is_finite`: 数值有效性检查.
- `_normalize_angle`: 角度归一化.
- `_timer_cb`: 根据全局/局部里程计差异计算 `map->odom` 平移与偏航, 持续广播 TF.

### launch/driver.launch

- 启动 `base_driver_node.py` 并配置串口/车体参数, 同时发布 `base_link->laser_link` 静态 TF.

### launch/navigation_core.launch

- 启动底盘驱动 + EKF:
  - `driver.launch`: 发布 `/odom` 与 `/gps/fix`.
  - `ekf_localization_node`: 生成 `/odometry/local`.
  - `navsat_transform_node`: 将 GPS 转为 `/odometry/gps`.
  - 可选全局 EKF 或 `global_odom_bridge` 生成 `/odometry/global`.

### CMakeLists.txt / package.xml

- Python 脚本安装; 依赖 `rospy`, `geometry_msgs`, `nav_msgs`, `tf`, `python3-serial`.

---

## osm_map_handler

### src/global_astar_node.cpp

- **`Node` / `CompareNode`**: A* 开放列表节点与优先队列比较器.
- **类 `GlobalAStarPlanner`**
  - `mapCallback`: 缓存 `/map` 栅格地图.
  - `odomCallback`: 缓存 `/odometry/global` 位姿.
  - `goalCallback`: 将起点/目标投影到栅格索引, 执行 A* 得到路径, 发布 `/global_astar_path`.
  - `runAStar`: 栅格 A* 搜索 (8 邻域, 代价加权).
  - `updateLocalGoal`: 从全局路径选取前瞻点, 发布 `/hybrid_astar_subgoal`.

### scripts/static_osm_map_server.py

- 将 OSM 瓦片拼接为大图, 转灰度并映射成 `OccupancyGrid` 发布 `/map`.
- 另存彩色纹理, 并发布用于 RViz 可视化的 Marker.

### scripts/osm_vector_planner.py

- 解析 OSM, 构建可通行道路图, 进行图搜索与路径重采样, 发布全局路径和子目标.

**关键函数**

- `latlon_to_xy`: 经纬度转换到局部平面.
- `load_graph`: 读取 OSM, 过滤道路类型, 构图并发布道路线段 Marker.
- `find_nearest_node` / `find_nearest_graph_point`: 找到最近道路节点或道路边投影点.
- `shortest_node_path`: 图上 A* 搜索节点路径.
- `resample_path_for_tracking`: 根据转角动态调整采样间距, 生成平滑路径点序列.
- `build_graph_path`: 结合起终点投影点与图路径拼接最终路径.
- `publish_initialpose`: 发布 `initialpose` 供定位对齐.
- `publish_pose_markers`: 发布当前位姿/路径/起点/GP S点等 Marker.
- `publish_snap_debug_markers`: 发布吸附起终点及连线.
- `publish_current_snap_marker`: 持续显示当前位置吸附到道路的点.
- `goal_callback`: 接收目标, 执行吸附 + A* + 重采样, 发布 `/osm_global_path`.
- `timer_callback`: 按前瞻距离发布 `/hybrid_astar_subgoal`.

### scripts/osm_vector_display.py

- 解析 OSM 路网并发布可视化 Marker (仅用于 RViz 显示).

### scripts/osm_tile_stitcher.py

- 将指定 zoom 的瓦片拼接为局部网格地图, 发布 `/map` 给 Hybrid A* 使用.

### scripts/gps_goal_bridge.py

- 订阅 `NavSatFix` 经纬度目标, 转换为 `map` 坐标 `PoseStamped`, 发布 `/move_base_simple/goal` 并可视化 Marker.

### CMakeLists.txt / package.xml

- 编译 `global_astar_node` 并声明 ROS 依赖.

---

## pcd2pgm_package/pcd2pgm

### README.md

- 点云 PCD 转二维栅格地图的简要说明.

### src/pcd2pgm.cpp

- `main`: 读取参数与 PCD 文件, 依次执行直通滤波与半径滤波, 生成 `OccupancyGrid` 并持续发布.
- `PassThroughFilter`: 基于 Z 轴阈值的直通滤波, 输出高度范围内点云.
- `RadiusOutlierFilter`: 半径离群点剔除, 清理孤立噪点.
- `SetMapTopicMsg`: 依据点云范围与分辨率构建 `OccupancyGrid` 数据.

### launch/run.launch

- 配置 PCD 文件路径、滤波阈值与地图分辨率, 启动 `pcd2pgm` 节点.

### CMakeLists.txt / package.xml

- 依赖 `pcl_ros`, `nav_msgs`, `sensor_msgs` 等, 构建可执行 `pcd2pgm`.

---

## urdf

### yahboomcar_R2.urdf.xacro

- 定义车体各 link/joint 的几何、惯性与碰撞.
- 描述转向关节、车轮关节、相机/激光/IMU 等传感器挂载位置.
- 含 `transmission` 配置, 主要启用前轮转向关节.

### yahboomcar_R2.gazebo.xacro

- Gazebo 物理与插件配置: 轮胎摩擦系数、Ackermann 驱动插件、激光/相机/IMU 传感器插件.

---

## hybrid-a-star-annotation (hybrid_astar)

### README.md

- 说明 Hybrid A* 的背景、论文与参考实现来源.

### src/main.cpp

- `message`: 控制台输出辅助函数.
- `main`: 初始化 ROS, 创建 `Planner` 并调用 `plan()`.

### include/planner.h / src/planner.cpp

**类 `Planner`**

- `Planner()`: 初始化订阅/发布与模式(静态/动态地图).
- `initializeLookups()`: 生成 Dubins/碰撞查找表.
- `setMap()`: 更新地图, 初始化 Voronoi, 动态模式下获取 TF 并触发规划.
- `setStart()`: 接收 RViz 起点, 更新有效性并触发规划.
- `setGoal()`: 接收 RViz 终点, 更新有效性并触发规划.
- `plan()`: 分配 2D/3D 节点数组, 调用 `Algorithm::hybridAStar`, 追踪与平滑路径并发布.

### include/algorithm.h / src/algorithm.cpp

- `Algorithm::hybridAStar`: Hybrid A* 主循环, 使用 open/closed 集合与 motion primitives 搜索.
- `aStar`: 2D A* 用于 holonomic-with-obstacles 启发式.
- `updateH`: 组合 Dubins/Reeds-Shepp/2D A* 启发式并写入 3D 节点.
- `dubinsShot`: 尝试用 Dubins 直连目标, 成功则提前返回.

### include/node3d.h / src/node3d.cpp

**类 `Node3D`**

- `updateG`: 根据前进/倒车与方向改变惩罚更新 g 值.
- `createSuccessor`: 基于 `dx/dy/dt` 生成后继节点.
- `isOnGrid` / `isInRange` / `operator==`: 网格与比较逻辑.

### include/node2d.h / src/node2d.cpp

**类 `Node2D`**

- `updateG` / `updateH`: 更新代价与启发式.
- `createSuccessor`: 8 邻域后继.
- `isOnGrid` / `operator==`: 网格与比较逻辑.

### include/constants.h

- 算法超参数与车辆尺寸/运动学设置(离散角度数、cellSize、惩罚项、Dubins 参数等).

### include/helper.h

- `normalizeHeading`, `normalizeHeadingRad`, `toDeg`, `toRad`, `clamp`: 角度与数值工具.

### include/collisiondetection.h / src/collisiondetection.cpp

**类 `CollisionDetection`**

- `isTraversable`: 基于碰撞查找表测试节点可通行性.
- `configurationTest`: 对车辆占据的栅格做碰撞检查.
- `updateGrid`: 更新栅格指针.

### include/path.h / src/path.cpp

**类 `Path`**

- `updatePath`: 将 3D 节点路径转为 RViz 线段/节点/车体 Marker.
- `addSegment` / `addNode` / `addVehicle`: 构建可视化数据.
- `clear` / `publishPath*`: 清理并发布路径.

### include/smoother.h / src/smoother.cpp

**类 `Smoother`**

- `tracePath`: 从目标回溯生成节点序列.
- `smoothPath`: 结合障碍项/曲率项/平滑项进行梯度下降平滑.
- `obstacleTerm` / `curvatureTerm` / `smoothnessTerm`: 平滑代价计算.

### include/visualize.h / src/visualize.cpp

**类 `Visualize`**

- `publishNode3DPose(s)` / `publishNode2DPose(s)`: 发布搜索过程节点.
- `publishNode3DCosts` / `publishNode2DCosts`: 发布代价热力图.
- `clear`: 清理可视化缓存.

### include/lookup.h

- `dubinsLookup`: 生成 Dubins 代价查找表.
- `collisionLookup`: 生成车体占据格查找表.

### include/vector2d.h / include/point.h

- `Vector2D`: 2D 向量运算.
- `IntPoint`: 整型点结构.

### include/gradient.h

- `ColorGradient`: 代价值到颜色映射的梯度工具.

### include/dynamicvoronoi.h / src/dynamicvoronoi.cpp

- 动态距离场与 Voronoi 图更新(占据/清除/更新/剪枝/可视化).

### include/dubins.h / src/dubins.cpp

- Dubins 曲线生成与采样: `dubins_init`, `dubins_path_length`, `dubins_path_sample`, `dubins_path_sample_many`, `dubins_extract_subpath`.

### src/tf_broadcaster.cpp

- 发布 `odom->map` 与 `map->path` 的 TF 变换.

### launch/manual.launch / velodyne_arlanda.launch

- 启动 `hybrid_astar` 与 `tf_broadcaster`, 以及地图/rviz 或 rosbag 回放.

### EwingKang-Dubins-Curve-For-MATLAB

- `dubins_core.m`: 计算 Dubins 曲线参数.
- `dubins_curve.m`: 生成并采样 Dubins 曲线, 可视化输出.

### CMakeLists.txt / package.xml / dynamicvoronoi 子包

- 声明依赖与构建目标(含 OMPL 与 dynamicvoronoi).

---

## FAST_LIO (fast_lio)

### README.md

- FAST-LIO/FAST-LIO2 的算法介绍、依赖与运行说明.

### msg/Pose6D.msg

- 描述 IMU 预积分状态(加速度/角速度/速度/位置/旋转矩阵 + 时间偏移).

### src/preprocess.h / src/preprocess.cpp

**类 `Preprocess`**

- `set`: 配置特征提取、雷达类型、盲区与滤波参数.
- `process`: 针对 Livox `CustomMsg` 或通用 `PointCloud2` 的入口.
- `avia_handler` / `oust64_handler` / `velodyne_handler` / `sim_handler`: 不同雷达点云解析与时间戳处理.
- `give_feature`: 平面/边缘特征分类与筛选.
- `plane_judge` / `edge_jump_judge`: 平面与跳变边缘判定逻辑.

### src/IMU_Processing.hpp

**类 `ImuProcess`**

- `Reset`: 重置 IMU 初始化状态.
- `set_extrinsic` / `set_gyr_cov` / `set_acc_cov` / `set_gyr_bias_cov` / `set_acc_bias_cov`: 设置外参与噪声.
- `IMU_init`: 估计重力/偏置并初始化 EKF.
- `UndistortPcl`: 结合 IMU 轨迹去畸变点云.
- `Process`: IMU 初始化与去畸变主入口.

### src/laserMapping.cpp

- `SigHandle`: 退出信号处理.
- `dump_lio_state_to_log`: 记录当前状态用于日志.
- `pointBodyToWorld*` / `RGBpointBodyToWorld` / `RGBpointBodyLidarToIMU`: 点云坐标系转换.
- `points_cache_collect` / `lasermap_fov_segment`: 局部地图裁剪与 KD 树维护.
- `standard_pcl_cbk` / `livox_pcl_cbk` / `imu_cbk`: 点云与 IMU 回调.
- `sync_packages`: 同步 IMU 与点云测量组.
- `map_incremental`: 将当前帧增量加入地图 KD 树.
- `publish_frame_world` / `publish_frame_body` / `publish_effect_world` / `publish_map`: 发布点云与地图.
- `publish_odometry` / `publish_path`: 发布里程计与轨迹.
- `h_share_model`: 构建观测残差与 Jacobian 供 EKF 优化.
- `main`: ROS 参数读取, 订阅/发布初始化, 进入主循环.

### include/common_lib.h

- 定义数学/类型别名、常量、`MeasureGroup`/`StatesGroup` 结构体与平面估计工具.

### include/use-ikfom.hpp

- 定义 `state_ikfom`/`input_ikfom`/`process_noise_ikfom` 的流形状态与噪声; 提供 `get_f`, `df_dx`, `df_dw`, `SO3ToEuler`.

### include/so3_math.h / include/Exp_mat.h

- SO(3) 指数/对数映射与旋转矩阵工具函数.

### include/ikd-Tree/ikd_Tree.h

- `KD_TREE` 动态 KD-Tree: 增删/搜索/重建与多线程维护接口.

### CMakeLists.txt / package.xml

- 依赖 PCL/Eigen/IKFoM/livox_ros_driver2, 构建 `fastlio_mapping`.

---

## livox_ros_driver2

### README.md

- ROS1/ROS2 通用驱动说明, 包含话题格式选择、发布频率、多雷达多话题等参数说明.

### msg/CustomPoint.msg

- 自定义点云点结构: `offset_time`(相对时间), `x/y/z`, `reflectivity`, `tag`, `line`.

### msg/CustomMsg.msg

- 自定义点云消息: `header`, `timebase`, `point_num`, `lidar_id`, `points` 数组.

### src/livox_ros_driver2.cpp

- ROS1 主入口: 初始化 ROS, 读取参数(`xfer_format`, `multi_topic`, `data_src`, `publish_freq`, `frame_id`, bag 开关), 约束发布频率, 创建 `Lddc`, 初始化 `LdsLidar` 并启动点云/IMU 轮询线程.
- ROS2 组件节点: 声明并读取参数, 初始化 `Lddc` + `LdsLidar`, 启动轮询线程, 通过 `RCLCPP_COMPONENTS_REGISTER_NODE` 注册组件.

### src/driver_node.h / src/driver_node.cpp

- `DriverNode`: ROS1/ROS2 节点封装, 持有 `Lddc` 与轮询线程.
- `GetNode`: 返回自身引用(ROS1/ROS2 统一接口).
- 析构函数: 触发 `lds_->RequestExit()`, 设置退出信号, `join` 两个轮询线程.
- `PointCloudDataPollThread` / `ImuDataPollThread`: 周期调用 `Lddc::DistributePointCloudData/DistributeImuData`.

### src/lds.h / src/lds.cpp

- `Lds`: 雷达数据源抽象与缓存管理.
- `StorageImuData`: 将 IMU 数据按雷达类型/句柄映射到索引, 入队并触发 IMU 信号量.
- `StoragePointData` / `StorageLvxPointData`: 从 `PointFrame` 提取点云包, 通过 `CacheIndex` 找到索引并入队.
- `PushLidarData`: 初始化队列(按发布频率计算大小), 入队并触发点云信号量.
- `ResetLds` / `ResetLidar`: 重置设备队列与状态.
- `RequestExit` / `IsAllQueueEmpty` / `IsAllQueueReadStop`: 退出与队列空判断.

### src/lds_lidar.h / src/lds_lidar.cpp

- `LdsLidar`: Raw LiDAR 数据源实现(单例).
- `InitLdsLidar`: 绑定全局指针, 解析配置, 设置回调, 启动数据流.
- `ParseSummaryConfig`: 读取 `lidar_summary_info.lidar_type`.
- `InitLidars` / `InitLivoxLidar`: 解析配置文件, 初始化 Livox SDK, 填充 `LidarDevice` 与外参, 注册信息回调.
- `SetLidarPubHandle`: 绑定点云/IMU 回调并设置发布频率.
- `DeInitLdsLidar` / `PrepareExit`: 释放 SDK 资源.

### src/lddc.h / src/lddc.cpp

- `Lddc`: 数据分发中心, 负责从 `Lds` 队列取数据并发布到 ROS/rosbag.
- `RegisterLds`: 绑定数据源.
- `DistributePointCloudData` / `DistributeImuData`: 等待信号量, 轮询各雷达队列.
- `PollingLidarPointCloudData` / `PollingLidarImuData`: 按格式分别调用发布函数.
- `InitPointcloud2MsgHeader` / `InitPointcloud2Msg` / `PublishPointcloud2Data`: 组装 `PointCloud2` 并发布/写 bag.
- `InitCustomMsg` / `FillPointsToCustomMsg` / `PublishCustomPointData`: 组装自定义点云格式.
- `InitPclMsg` / `FillPointsToPclMsg` / `PublishPclData`: ROS1 `pcl::PointCloud` 发布(ROS2 不支持).
- `InitImuMsg` / `PublishImuData`: IMU 消息构造与发布.
- `GetCurrentPublisher` / `GetCurrentImuPublisher`: 单话题或按 IP 生成多话题发布器.
- `CreateBagFile`: 创建 rosbag 输出文件.

### src/parse_cfg_file/parse_cfg_file.*

- `ParseCfgFile::ParseSummaryInfo`: 解析 `lidar_summary_info.lidar_type`.

### src/parse_cfg_file/parse_livox_lidar_cfg.*

- `LivoxLidarConfigParser::Parse`: 解析 `lidar_configs` 列表.
- `ParseUserConfigs`: 读取 `ip`, `pcl_data_type`, `pattern_mode`, `blind_spot_set`, `dual_emit_en`, `extrinsic_parameter`.
- `ParseExtrinsics`: 读取外参 `roll/pitch/yaw/x/y/z`.

### src/comm/comm.*

- 定义常量、枚举与核心数据结构: `PointFrame`, `PointPacket`, `StoragePacket`, `LidarDevice`, `ExtParameter` 等.
- `IsFilePathValid` / `CalculatePacketQueueSize` / `IpNumToString` / `IpStringToNum` / `ReplacePeriodByUnderline`: 通用工具函数.

### src/comm/ldq.*

- 点云环形队列操作: `InitQueue`, `DeInitQueue`, `QueuePop`, `QueuePushAny`, `QueueIsEmpty/Full` 等.

### src/comm/lidar_imu_data_queue.*

- 线程安全 IMU 队列: `Push`, `Pop`, `Empty`, `Clear`.

### src/comm/semaphore.*

- 简单计数信号量: `Signal` / `Wait`.

### src/comm/cache_index.*

- 句柄到索引映射: `GetFreeIndex`, `GetIndex`, `LvxGetIndex`, `ResetIndex`.

### src/comm/pub_handler.*

- `PubHandler`: SDK 原始包观察者, 缓存原始包队列, 线程化解包与按发布周期拼帧.
- `SetPointCloudConfig`: 配置发布周期并启动处理线程.
- `OnLivoxLidarPointCloudCallback`: 接收 SDK 数据包, 组装 `RawPacket`, 推入处理队列.
- `RawDataProcess`: 取包, 分配 `LidarPubHandler`, 调用点云解析.
- `CheckTimer`: 根据时间同步与发布频率决定何时发布 `PointFrame`.
- `LidarPubHandler::Process*`: 解析高/低精度笛卡尔或球坐标点, 应用外参补偿并生成 `PointXyzlt`.

### src/call_back/lidar_common_callback.*

- `OnLidarPointClounCb`: 将 `PointFrame` 推入 `LdsLidar` 缓存.
- `LidarImuDataCallback`: 将 IMU 数据推入 `LdsLidar` 缓存.

### src/call_back/livox_lidar_callback.*

- `LidarInfoChangeCallback`: 设备上线后应用配置(点云格式/扫描模式/盲区/双发射/外参)并切换工作模式.
- `SetDataTypeCallback` / `SetPatternModeCallback` / `SetBlindSpotCallback` / `SetDualEmitCallback`: 异步配置结果处理与重试.
- `SetAttitudeCallback`: 外参写入结果处理.
- `EnableLivoxLidarImuDataCallback`: IMU 使能结果处理.

---

## Livox-SDK2

### README.md

- SDK2 总览(支持 HAP/Mid-360), 构建要求与安装方式, 以及三类示例程序(`livox_lidar_quick_start`, `logger`, `multi_lidars_upgrade`).
- 说明配置文件结构: `master_sdk`, `lidar_log_enable`, 以及 `HAP`/`MID360` 的 `lidar_net_info` 与 `host_net_info` 端口/IP 配置.

### CMakeLists.txt

- 设置 C++11, 编译 `sdk_core` 与 `samples` 子项目.

### include/livox_lidar_api.h

- SDK C 风格 API 声明: 初始化/反初始化、数据/IMU/状态回调注册、点云观察者、参数设置、工作模式控制、升级与日志接口等.

### include/livox_lidar_def.h

- 设备类型、点云数据类型、工作模式、命令返回码等枚举定义.
- 定义以太网包结构与点类型(`LivoxLidarEthernetPacket`, `LivoxLidarCartesianHighRawPoint` 等).

### include/livox_lidar_cfg.h

- 不同平台的 IO 复用宏开关(EPOLL/SELECT/KQUEUE/POLL).

### sdk_core/livox_lidar_sdk.cpp

- `LivoxLidarSdkInit`: 解析配置文件/参数校验, 初始化 `DeviceManager` 与日志, Windows 下初始化 WSA.
- `LivoxLidarSdkUninit`: 释放 `DeviceManager`, `DataHandler`, `GeneralCommandHandler`, 日志系统与 WSA.
- API 封装层: 将外部 API 转发至 `CommandImpl`, `DataHandler`, `GeneralCommandHandler`, `UpgradeManager`, `LoggerManager`.

### sdk_core/device_manager.h / sdk_core/device_manager.cpp

- 管理检测/命令/数据通道的 socket 与 IO 线程.
- `Init`: 根据配置初始化日志、命令处理与数据处理模块, 创建通道与检测线程.
- `CreateDetectionChannel` / `CreateCommandChannel` / `CreateDataChannel`: 按配置建立端口监听与广播通道.
- `DetectionLidars` / `Detection`: 广播发现雷达.
- `OnData`: 接收 UDP 数据, 分流到 `DataHandler`/`GeneralCommandHandler`/Logger/Debug 处理.
- `HandleDetectionData`: 处理发现结果并触发设备状态变化.
- `GetLivoxLidarInternalInfoCallback` / `AddViewLidar`: 视图模式下解析端口并动态创建数据通道.
- `SendCommand` / `SendLoggerCommand`: 发送控制/日志命令.
- `Destory`: 关闭 socket、停止线程并清理映射表.

### sdk_core/parse_cfg_file.h / sdk_core/parse_cfg_file.cpp

- 解析 JSON 配置文件, 支持 `HAP`/`MID360` 设备段.
- `ParseNewLidarCfg`/`ParseOldLidarCfg`: 兼容 `host_net_info` 数组/对象两种配置格式.
- `ParseLidarNetInfo` / `ParseHostNetInfo`: 解析端口与 IP 字段.

### sdk_core/params_check.h / sdk_core/params_check.cpp

- 配置合法性检查: IP 冲突检测、多播 IP 合法性、Mid360 端口强制校验.

