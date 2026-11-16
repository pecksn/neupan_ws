# 自维护功能包分类

按用途分类，不按编程语言分类；不新增 `python/`，不为了统一外观复制代码或增加空目录。

| 目录 | 放什么 |
|---|---|
| `src/` | 核心实现和可复用模块，Python、C++ 都可以 |
| `scripts/` | 可直接执行的 ROS 节点和工具脚本 |
| `Test/tests/` | 自维护代码的单元、回归及隔离进程测试（集中管理） |
| `launch/`、`config/` | 系统装配和参数 |
| `msg/`、`srv/` | ROS 消息和服务定义（需要时才建） |
| `example/` | 实车配置与模型；无关的 Gazebo/TEB/RVO 对照已移除 |

## 业务包

| 功能包 | 职责与源码位置 |
|---|---|
| `osm_map_handler` | OSM 全局 A*、返程衔接和相机参考路径；核心在 `src/`，节点入口在 `scripts/` |
| `neupan_ros` | 局部优化和避障控制；`src/neupan_core.py`、`src/neupan_diagnostics.py` 为实现，`scripts/neupan_node.py` 为入口 |
| `robot_base_driver` | 底盘、定位保护、控制选择与日志；可执行节点在 `scripts/`，公共时间模块在 `src/gnss_timebase.py`，回归在 `Test/tests/` |
| `hybridnets_receiver` | GPU 视频与 JSON 接收、叠加验证、视频录制；节点在 `scripts/`，公共协议模块在 `src/hybridnets_binding.py` |
| `usb_camera` | 相机启动、内外参标定和离线验证；工具在 `scripts/`，标定参数在 `config/` |
| `vehicle_bag_recorder` | rosbag 录制；节点在 `scripts/`，录制话题和分包规则在 `config/` |
| `Test`（ROS 包名 `test_motion_tools`） | `scripts/` 为人工测试和采集工具，`tests/` 为自动回归；部分人工测试会启动车辆，不能当无硬件回归执行 |

OSM 公共模块直接平铺在 `osm_map_handler/src/`，不再创建内层同名目录；文件名用 `osm_` 前缀避免跨包冲突。`scripts/camera_right_path.py` 只是入口，唯一的核心实现在 `src/osm_camera_path_refinement.py`，入口和实现不同名。NeuPAN 的核心 Python 源码继续留在 `src/`，不机械搬进 `scripts/`。

项目自编测试统一到 `Test/`，厂家/第三方原带测试及其配套数据保留原目录。已有布局合理的业务源码不搬文件。FAST-LIO、Livox、Wheeltec、NMEA 消息、共享 Hybrid A* 及其第三方依赖保留原目录；现有 C++ 辅助包也不因分类而迁移。

## 日志职责

业务节点计算并发布状态，`robot_base_driver/scripts/system_run_logger.py` 统一写入运行 JSONL；`vehicle_bag_recorder` 单独录制 ROS 话题。`Test/scripts/analyze_bridge_reference_log.py` 只离线分析已有日志，不新增常驻记录节点。

## 业务代码格式与注释

Python 使用根目录 `pyproject.toml` 的 Black 配置：4 空格缩进、88 字符目标行宽、Python 3.8 语法；本次保留字符串引号写法。C++ 使用根目录 `.clang-format`：4 空格缩进、88 字符目标行宽，不自动重排 include。不要对厂家/第三方目录执行全仓库批量排版。

英文注释优先解释职责边界、坐标系及正负号、物理单位、源时间与接收时间、线程/锁约束、拒绝条件和回退原因；不把配置偏好写成强制控制保证，不把离线验证写成实车验收。已有协议字符串、日志字段、参数默认值与控制逻辑不随格式整理改变。

2026-11-16 注释补全范围：自维护业务及实验入口的 45 个 Python 文件、4 个 C++ 文件和 5 个 Shell 文件。Python 共 598 个类/函数：保留 105 处已有文档，补充 493 处英文说明，包括嵌套辅助函数；每个文件均注明模块职责。C++ 补充类型/函数职责，Shell 注明执行环境、进程归属及副作用。仅纳入 Wheeltec 包中自维护的 4 个 GNSS 辅助模块，厂家实现、第三方依赖及测试文件不属于此次逐函数补注范围。逐文件覆盖清单和语法等价检查依据保存在 `tracker_logs/review_20261116/business_comments/coverage.json`。

## 编译和安全回归

进入 `Neupan`、按 `e` 后，在 ROS1 `lxw` 环境执行：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
catkin_make --pkg hybrid_astar neupan_ros robot_base_driver osm_map_handler -j2
source devel/setup.bash
python -m unittest discover -s src/Test/tests -v
NEUPAN_CPP_SMOKE=1 NEUPAN_ROS_SMOKE=1 python src/Test/tests/test_outdoor_start_connection.py -v
```

上述回归不启动底盘驱动；ROS 进程测试使用私有 master、假传感器及隔离速度话题。原 `roslaunch`/`rosrun` 的包名和节点名不变；目录整理不改变车辆参数，新增室外局部 Hybrid A* 见 [OSM 说明](osm_map_handler/README.md)。视频仍放工作区 `videos/`，日志仍放 `tracker_logs/`，rosbag 仍放 `rosbags/`。

2026-10-16 分类后验证：ROS1 编译通过，定位/运动保护 23 项与路径/入口/隔离进程 43 项回归全部通过；室内、室外 launch 配置解析通过。迁移的 9 个底盘节点与原文件内容逐一校验一致，仅改变位置和可执行权限；旧目录的字节码缓存已清理。OSM 增加入口/实现分名和平铺目录检查，避免再次出现同名嵌套。

共享 Hybrid A* 接入后补测：路径/搜索/进程 57 项与定位/运动保护 23 项，共 80 项通过；室内简单直线入口可用，对应历史仿真报告已按2026-11-06清理范围移除。室内历史运动原语保持不变，本次不等于完成室内任意目标/场景验收。
