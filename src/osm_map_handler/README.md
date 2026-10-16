# OSM 地图与全局参考路径

负责地图路网、全局 A*、室外返程衔接和相机靠右路径修正；不发布车辆速度。

```text
osm_map_handler/
├── src/
│   ├── global_astar_node.cpp            OSM 路网与全局 A*
│   ├── outdoor_hybrid_worker.cpp        室外地图适配，调用共享 C++ 搜索
│   ├── osm_hybrid_search.py             独立搜索进程的请求、超时与取消
│   ├── osm_outdoor_start_connection.py  返程连接段、道路与扫描准入
│   ├── osm_camera_path_refinement.py    相机投影、道路段缓存与参考路径修正
│   └── osm_camera_path_guard.py         摄像头/原路线选择与心跳策略
├── scripts/
│   ├── camera_right_path.py            ROS 节点入口，仅调用上面的实现模块
│   ├── gps_goal_bridge.py              GPS 目标桥接节点
│   └── osm_alignment_analyzer.py       地图分析工具
├── launch/camera_right_path.launch
└── setup.py                            catkin 平铺 Python 模块注册
```

`neupan_ros` 消费准备好的参考路径，负责局部优化、避障和速度控制。心跳监督仍在 NeuPAN 消费进程中独立执行，代码归属迁移不改变失联回退机制。`robot_base_driver` 保留底盘、定位、控制切换和日志；其中 `global_path_tracker.py` 是输出速度的跟踪控制器，不是全局规划器。

公共模块直接放在 `src/`，不再嵌套同名文件夹，也不新增 `python/`。模块用 `osm_` 前缀避免与其他功能包重名，通过 catkin 安装后跨包导入。入口 `camera_right_path.py` 只负责启动，`osm_camera_path_refinement.py` 保留唯一一份功能实现；原节点名、话题与启动命令不变。

通用 C++ 混合 A* 由独立 ROS 包 `hybrid_astar`（`src/hybrid-a-star-annotation`）导出 `HYAS` 库，室内入口与室外 worker 共用搜索核心，不复制第二套 Python 算法。室外仅在路线起始方向/位置不匹配时按需搜索前进连接段，正常路段仍走 OSM A*，不重算整条路线。

室外采用 16×16 m、0.1 m 分辨率的局部图，按车辆外形、至少 1.85 m 转弯半径和 5 种前进曲率搜索；未知回波不能当空地。独立进程搜索时停车，默认 `outdoor_connection_budget_sec:=1.0`、`outdoor_connection_max_iterations:=12000`。返回后用最新、采集时刻 TF 对齐的扫描再次检查完整连接段；车辆移动、观测不足、超时或失败均拒绝，不自动回退反向参考线。

全系统默认 `outdoor_connection_planner:=hybrid_astar`；需要旧方案对照时显式设置 `outdoor_connection_planner:=dubins`。不要为此打开旧的 `outdoor_use_hybrid_astar`：它是整段栅格规划分支，不是本地掉头开关。原室内车辆/搜索默认参数及倒车选择保留；共享核心另外补上迭代耗尽失败、边界检查和未收到地图时拒绝目标的保护。

全系统启动命令、节点名和话题不变；室外连接算法默认改为上述 Hybrid A*。单独启动相机路径节点改为：

```bash
roslaunch osm_map_handler camera_right_path.launch control_enabled:=false
```

单独启动仅输出验证路径，不会驱动车辆；需要已有定位、OSM 路线及摄像头 JSON。

首次迁移后在 ROS1 `lxw` 环境编译，使跨包导入与节点入口生效：

```bash
cd /home/nuc/lxw/neupan_ws
catkin_make --pkg hybrid_astar osm_map_handler neupan_ros robot_base_driver -j2
source devel/setup.bash
python -c 'from osm_outdoor_start_connection import StartRouteConnector; from osm_camera_path_guard import CameraPathGuard'
```

修改公共模块后重新执行 `catkin_make --pkg osm_map_handler` 并重启相关节点，使 devel 导出与源码一致。不在入口脚本中手动修改其他包的 `sys.path`。

状态看 `/neupan/route_connection`：`searching` 正在停车搜索，`connecting` 执行连接段，`following_route` 正常跟踪，`blocked` 需重新选择安全区域下发目标。`admission.search` 记录搜索耗时、展开节点与拒绝统计，`completion_scan_stamp_sec` 记录返回后复查扫描的时间。

回归入口：[`src/Test/tests/test_outdoor_start_connection.py`](../Test/tests/test_outdoor_start_connection.py)，覆盖隔离室内入口、室外搜索/取消/超时及扫描复查，不启动实车硬件。目录按 [功能包分类](../README.md) 管理。完整仿真见 [四场景报告](../../videos/outdoor_simulation/20261016_113623/summary.md)：宽路完成返程，窄路/未知/障碍均停车；不代表最小实车掉头宽度或实车验收。结果仍放 `videos/`，不放本包。
