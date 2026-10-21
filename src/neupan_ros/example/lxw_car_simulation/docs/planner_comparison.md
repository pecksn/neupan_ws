# LXW Gazebo 局部规划对比

这是 `neupan_ros/example/lxw_car_simulation` 内的局部规划对比功能，与原局部仿真、室外 OSM 仿真统一放在同一个目录。TEB 与 NeuPAN 共用车辆模型和底盘，只切换局部规划器。

## 运行

进入 ROS1：终端输入 `Neupan`、按 `e`，然后：

```bash
conda activate lxw
source /opt/ros/noetic/setup.bash
cd /home/nuc/lxw/neupan_ws
catkin_make --pkg neupan_ros -j2 -l2
source devel/setup.bash
```

TEB 仿真额外使用 `teb_local_planner`、`costmap_converter` 和 ROS navigation 的代价地图/插件接口。本机 ROS1 容器已安装；其他环境可安装 `ros-noetic-teb-local-planner` 和 `ros-noetic-costmap-converter`。TEB 的编译配置仅在本仿真目录的 `CMakeLists.txt` 中；缺少编译依赖时跳过适配器，也可用 `-DBUILD_LXW_GAZEBO_TEB=OFF` 关闭。重新启用时传 `-DBUILD_LXW_GAZEBO_TEB=ON`。

自动评估额外使用 `topic_tools/throttle`（`ros-noetic-topic-tools`），将 Gazebo 每个物理步发布的模型真值限频到30 Hz后送给 Python 评估节点；规划器仍使用原扫描和控制频率。对比世界的共同目标墙钟速率为0.5倍，控制、障碍时序与视频播放按仿真秒。此设置用于减少软件录屏负载，不作为实车实时性能证明。每轮记录扫描/里程计数据年龄及最大遥测采样间隔，扫描和指令超时阈值仍为0.5仿真秒。

对比master显式设置 `/gazebo/pub_clock_frequency=100`，避免 `empty_world.launch` 中未启用的时钟频率参数导致按每个物理步广播。每轮 `telemetry.bag` 由独立C++ rosbag记录时钟、扫描、里程计、模型真值及指令，用于核对消息连续性和完成状态，避免仅依赖Python评估采样判断。

对比世界使用仿真专用 `liblxw_sim_pose.so`，在Gazebo世界更新线程应用车辆及障碍位姿，避免50Hz调用外部位姿接口时反复暂停/恢复世界。需安装Gazebo开发依赖后重新 `catkin_make --pkg neupan_ros`。虚拟底盘仍按相同Ackermann运动学积分，不是轮胎动力学。普通局部/室外入口的默认位姿接口保持各自原设置。

常用文件：`launch/planner_comparison.launch` 装配场景，`config/comparison_neupan.yaml` 配置 NeuPAN，`config/comparison_teb_field.yaml` 配置 TEB；`config/comparison_teb.yaml` 保留原仿真参数组，`scripts/planner_comparison.py` 执行对比，`src/teb_sim_controller.cpp` 加载已安装的官方 TEB 插件。以上相对路径均以 `lxw_car_simulation/` 为起点。

两种规划器 × 三个固定时序动态场景，各重复两次，并录制真实 Gazebo 窗口：

```bash
rosrun neupan_ros planner_comparison.py run --scenario dynamics --teb-profile field --repeats 2 --record-gazebo
```

只运行 TEB 动态避障，并在当前显示器打开 Gazebo：

```bash
rosrun neupan_ros planner_comparison.py run \
  --planner teb --scenario crossing --repeats 1 --gui
```

NeuPAN 同样使用 `--planner neupan`。`--scenario dynamics` 包含 `lead_slow`、`lead_stop`、`crossing_once`；`all` 还包含 `open` 和 `static`，也可单独选择各场景。历史合作场景 `crossing` 与往返压测 `crossing_stress` 单独选择。`--timeout 75` 为每轮仿真时间上限。`--output-dir` 可指定本轮结果目录。`run` 总是为每轮创建独立 ROS/Gazebo master、独立日志目录并清理自己的进程。`--record-gazebo` 使用独立 Xvfb，不占用当前桌面。

`--neupan-profile adaptive` 为默认模式，沿用原局部仿真入口的自适应调参；`--neupan-profile fixed` 保留历史固定参数实验。参数表在 `config/comparison_neupan_adapter.yaml`，回归检查会与原 launch 的默认值逐项核对。每轮在 manifest 中标记模式，不能混用两种模式评价“正常配置”。`neupan_solver.jsonl` 保存逐次求解的输入点、DUNE采样点、模型距离、几何点距、扫描时效、参考状态和动作，供接入诊断；不将障碍真值传给规划器。

`--teb-profile field` 为默认参数组，来自用户确认的 `src/teb_test/config/teb_local_planner.yaml`。源校验值及适配差异在 `config/teb_field_adaptation.json`：覆盖坐标、里程计、车体、0.5 m/s限速和0.15 m终点阈值，实车文件不变。`--teb-profile baseline` 切换原仿真参数组。此处直接调用 TEB 插件，没有复现实车完整 `move_base` 恢复状态机、代价地图与传感器链，不能称为实测卡顿复现。

手动使用 RViz 的 `2D Nav Goal`，仅在专用仿真 master 中启动：

```bash
roslaunch neupan_ros planner_comparison.launch planner:=teb scenario:=static rviz:=true
```

目标坐标系为 `odom`，目标是后轴位置；Gazebo 中蓝色圆盘与橙红标杆表示目标，RViz 同时显示共同参考线、局部轨迹和扫描。

## 对比条件

| 项目 | 两者共用 |
|---|---|
| 起点 / 目标 | 后轴 `(0,0,0)` / `(8,0,0)` |
| 车体 | CAD 外观，矩形包络 0.95×0.75 m，中心在后轴前方 0.335 m |
| 运动学 | Ackermann，轴距 0.67 m，转角限制 ±0.35 rad，最小转弯半径约 1.85 m |
| 速度 / 控制周期 | ±0.5 m/s / 10 Hz，50 Hz 共用底盘 |
| 输入 | 同规格雷达和扫描处理链；每个场景中使用相同参考路径，路径坐标 SHA256 记录在结果中。车辆运动不同，实际扫描也会不同 |
| 到达验收 | 规划器报告完成、位置误差≤0.2 m、航向误差≤0.35 rad、速度≤0.1 m/s，稳定0.5 s；两者规划器内距离阈值0.15 m |
| 保护 | 扫描或指令超过0.5 s不更新即停车；非有限指令拒绝 |

原 `static` 场景保留直线穿过固定障碍的局部绕障基线。新版 `crossing` 中，两者使用同一条绕过固定障碍的平滑参考线：`y=1.35 sin²(πx/8)`，事先检查车体净距>0.2 m、曲率不超过1/1.85 m⁻¹。避免将固定障碍的全局参考线不可行问题混入动态局部规划。这个预设仅适用于标准起点 `(0,0)`、目标 `(8,0)`；任意手动目标仍沿用直线参考，未提供通用全局规划器。两者可倒车；没有添加各自不同的清图或脱困动作。TEB 使用 ROS 已安装的 `teb_local_planner/TebLocalPlannerROS`，开启 Ackermann 转角输出；不将角速度错当成转角。其代价地图将扫描栅格聚为障碍轮廓。NeuPAN 使用原 `neupan` 优化器及 `acker_robot_default_2` 权重，仿真适配层负责10 Hz调度、采集时刻 TF、路径/扫描与ROS消息转换；不改求解器或实车节点。

静态场景障碍为 `(3.8,0,r=0.4)` 与 `(5.6,-1.5,r=0.35)`。

固定时序动态场景均使用直线共同参考线、半径0.35 m圆柱，不根据车辆行为让路：

| 场景 | 障碍运动 |
|---|---|
| `lead_slow` | 从 `(2.4,0)` 以0.18 m/s向前慢行，至 x=10停止 |
| `lead_stop` | 从 `(2.8,0)` 以0.22 m/s前进，第6～10秒停车，随后继续至 x=10 |
| `crossing_once` | 从 `(4.2,3)` 以0.6 m/s单次横穿至 y=-3；车辆可在冲突区外等待 |

运行前将独立的安全绕行/等待路径作为离线几何及运动学可行性证据存入 `feasibility.json`，不传给规划器，也不将其作为 Gazebo 实际通过记录。停顿按已起步后速度低于0.03 m/s持续至少0.3 s判定，距离终点0.4 m内的停车排除；恢复阈值为0.08 m/s。启动等待单独统计，未恢复的停顿保留。


默认 `crossing` 场景包含半径0.4 m的固定绕行障碍 `(3.8,0)`，以及从 `(5.2,3)`、`(6.6,-3)` 出发的两个移动圆柱（半径0.35 m）。二者分别延迟4、6 s出发，以最高0.7 m/s单次横穿至道路另一侧。移动障碍根据车辆当前位姿/速度预测未来1.5 s的接近距离，可能小于0.6 m时暂停自己的路径进度，安全后从暂停处继续，不瞬移追赶原时间表。测试结束后障碍与车辆均停止。

这是双方交互场景：共用环境规则，实际障碍轨迹会随车辆行为变化，不宣称轨迹完全一致。`actor_yield_events` / `actor_yield_duration_s` 单独记录障碍让行，视频同步显示。车辆仍需绕过固定障碍，避免将全程直行且障碍单方面让路当成机器人动态避障能力的证明。环境内的移动障碍使用车辆真值模拟自身避碰感知，但这些真值不传给任一机器人规划器。

`crossing_stress` 保留原强制往返轨迹 `(3, 2.5 cos(2πt/20))` 和 `(5.2, -2.5 cos(2πt/26))`。它们不避让车辆，最高横向速度约0.79 / 0.60 m/s，只作为非合作横穿压测。旧录像中的动态段属于这个场景，不能用其碰撞结果直接评价正常交互避障能力。

动态障碍使用关闭重力的非静态运动学 link，避免连续移动静态模型时 Gazebo 窗口中的视觉位姿滞后；静态场景的圆柱仍使用静态模型。实际位移审计与窗口检查分别进行，不能只检查 `/gazebo/model_states` 就认定录屏显示正常。

## 输出与限制

结果分类保存到：

```text
videos/planner_comparison/日期_时间/
  comparison.md / comparison.json
  open_neupan_1/、open_teb_1/、static_neupan_1/……
    result.json       到达、间距、路长、停车、误差、优化调用耗时
    trajectory.jsonl  Gazebo真值车姿/障碍位置与局部规划
    manifest.json     独立master、参数校验值、模型、控制周期
    gazebo.mp4        真实Gazebo窗口录屏（加 --record-gazebo）
    overview.mp4      同轮真值轨迹回放，非GUI录屏
    console.log / recorder.log / ros_logs/
```

几何碰撞按实际矩形车体到圆柱的有符号距离及场地边界判定；侵入超过2 cm、扫描过期、里程计/Gazebo位置差>0.25 m或超时均不能算通过。记录实际运动障碍的累计位移，不以配置速度冒充运动。

虚拟底盘积分运动学并设置 Gazebo 位姿，不模拟轮胎、打滑、悬架或接触制动；不能据此得出实车碰撞安全结论。定位为仿真真值，传感器为二维射线切片，非真实 MID360/FAST-LIO。优化调用耗时包含各自优化器/插件工作，不包括完整感知流水线；有限场景、两次重复不能证明算法普遍优劣。TEB没有额外动态跟踪器；交互场景中的到达结果包括障碍主动让行的作用，不能作为单方面动态预测/避障能力通过的结论。

检查仿真几何和车辆积分：

```bash
python3 src/Test/tests/test_simulation_geometry.py
```

实际 Gazebo 指令/扫描超时停车与恢复检查（独立 master，不启动规划器）：

```bash
python3 src/Test/scripts/test_command_gate.py \
  --output-dir /home/nuc/lxw/neupan_ws/videos/planner_comparison/safety_check
```

`run` 完成后自动导出 `trajectories.png`、参考线/参数/动态位移审计 `input_audit.json`，以及按目标下发时刻对齐的左右轨迹视频 `*_comparison.mp4`。可用 `python3 src/neupan_ros/example/lxw_car_simulation/scripts/report_comparison.py 结果目录` 重新生成报告；真实 Gazebo 录屏仍在各轮 `gazebo.mp4`。

### 导出 1080p 并排视频

新录制的实验同时保存 `recording_timing.json` 和轨迹中的墙钟时间，可制作中文标题、左 TEB / 右 NeuPAN 的视频：

```bash
rosrun neupan_ros export_comparison_video.py \
  /home/nuc/lxw/neupan_ws/videos/planner_comparison/20261019_adaptive_full_comparison --all-repeats
```

输出到该轮 `presentation/`，包括 `teb_neupan_gazebo_1080p.mp4`（真实窗口和下方轨迹）、`teb_neupan_trajectory_1080p.mp4`（等比例轨迹回放）、预览图及视频来源/分段信息。均为 1920×1080、15 fps、H.264/yuv420p、方形像素，画面等比例缩放。

默认视频覆盖全部匹配场景的第1轮，`--all-repeats` 保留全部重复；不挑选通过的案例；先结束的一侧冻结，失败原因保留在结尾。录屏通过 X11 首帧时间戳与实际记录的仿真/墙钟时间映射对齐，窗口本身仍存在渲染延迟。原始录屏保持原样。旧实验缺少时间侧录时，可加 `--view trajectory` 只导出按仿真时间对齐的轨迹视频。

`run` 遇到已有数据的试验子目录会拒绝运行，请使用新的 `--output-dir`，避免旧 `result.json` 导致提前结束或混用新旧录屏。报告中的测试轮数由实际结果生成。

TEB 参数含义依据[官方 Noetic 参数定义](https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/cfg/TebLocalPlannerReconfigure.cfg)，执行时以本机安装版本为准。

## 2026-10-19 最新完整对比

[最新视频及检查说明](../../../../../videos/planner_comparison/20261019_adaptive_full_comparison/presentation/README.md)：原仿真自适应 NeuPAN 对比用户确认实车参数的 TEB 仿真适配副本。三个固定时序场景各两轮，共12轮，全部保留在两份1080p视频中。

| 场景 | NeuPAN 到达 | TEB 到达 | NeuPAN 中途停顿 | TEB 中途停顿 |
|---|---|---|---|---|
| 前方慢行 | 2/2 | 2/2 | 0 | 2次，0.8秒 |
| 停车再起步 | 2/2 | 1/2 | 0 | 2次，4.15秒 |
| 单次横穿 | 2/2 | 2/2 | 0 | 0 |

NeuPAN停车再起步第2轮起步前出现最大0.28秒消息间隔，未通过全程连续性审计；终点停车及位置检查通过。TEB未完成的一轮在接触附近触发约0.272米的Gazebo/里程计偏离保护。两项均保留，不能把到达记录当作全部严格审计通过。

共同底盘改用世界更新线程传送运动学位姿，减少原外部位姿接口暂停/恢复世界造成的消息间断；不是轮胎动力学验证。实际Gazebo指令过期、扫描过期停车及恢复独立复测通过；与视频编码并行的首次车体位移检查失败，原因尚未唯一定位，详见最新说明。结果只适用于本次配置和有限场景，不作为实车卡顿原因或算法普遍优劣结论。

原局部入口的自适应规则现为对比默认配置，历史fixed模式需显式选择。旧的失败视频、结果与中断诊断已按用户要求删除，见本轮清理记录；不再把历史fixed视频当作最新结果。

保留的成功参考：[合作横穿](../../../../../videos/planner_comparison/20261019_interactive_reference/presentation/README.md)（包含障碍主动让行），[目录迁移空场检查](../../../../../videos/planner_comparison/20261019_1430_layout_check/comparison.md)。
