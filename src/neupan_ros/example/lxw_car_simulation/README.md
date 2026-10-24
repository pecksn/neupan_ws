# LXW 车辆仿真（ROS1 / Gazebo）

这是车辆 Gazebo 仿真的统一目录，包含 NeuPAN、TEB 和室外 OSM 场景。`neupan_ros` 是所属 ROS 包名，不表示此目录只能运行 NeuPAN。原先新增的 `src/lxw_planner_sim` 已合并到这里。

文件按资源类型归类：`comparison_` / `planner_comparison` 表示两种规划器共用的对比功能，`outdoor_` 表示室外仿真。测试结果分类放在工作空间的 `videos/` 下。

## 目录

```text
lxw_car_simulation/
├── launch/   启动入口与组件装配
├── scripts/  虚拟底盘、规划器适配、对比/室外测试工具
├── src/      TEB 仿真适配器（加载官方 TEB 插件）
├── config/   原局部、NeuPAN/TEB 对比与 RVO 参数
├── maps/     室外 OSM 测试路网
├── worlds/   Gazebo 场景（含 outdoor_roads.world）
├── urdf/     车辆模型（含 outdoor_lxwcar.urdf.xacro）
├── rviz/     显示配置（含 outdoor_simulation.rviz）
├── meshes/   共用车辆 CAD 外观
├── models/   共用箱体、圆柱障碍物
├── tests/    仿真几何、停车保护与原入口回归检查
├── docs/     规划器对比说明
├── CMakeLists.txt  可选 TEB 仿真编译配置
└── README.md
```

| 文件 | 用途 |
|---|---|
| [launch/lxw_car_simulation.launch](launch/lxw_car_simulation.launch) | 原室内/局部仿真入口 |
| [launch/planner_comparison.launch](launch/planner_comparison.launch) | 统一对比场景，通过 `planner:=neupan` / `planner:=teb` 切换规划器 |
| [launch/neupan_lxwcar_gazebo.launch](launch/neupan_lxwcar_gazebo.launch) | 上述入口调用的 Gazebo 与控制组件装配 |
| [launch/outdoor_simulation.launch](launch/outdoor_simulation.launch) | 室外 OSM 往返仿真装配，由测试脚本在独立 master 中启动 |
| [scripts/outdoor_simulation.py](scripts/outdoor_simulation.py) | 室外一键运行、传感器桥接、自动测试和录制 |
| [scripts/planner_comparison.py](scripts/planner_comparison.py) | 两种规划器自动对比、指标采集与录制 |
| [scripts/export_comparison_video.py](scripts/export_comparison_video.py) | 1080p 左右对比视频：真实 Gazebo 窗口与等比例轨迹 |
| [scripts/ackermann_sim_driver.py](scripts/ackermann_sim_driver.py) | 各仿真共用的运动学虚拟底盘 |
| [scripts/publish_demo_goal.py](scripts/publish_demo_goal.py) | 原局部仿真的演示目标发布 |
| [scripts/gazebo_goal_marker.py](scripts/gazebo_goal_marker.py) | Gazebo 目标标记，无碰撞、不进入雷达回波 |

## ROS1 环境

先输入 `Neupan`、按 `e`，再执行：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
```

## 快速启动 NeuPAN / TEB

迁移后先编译一次：

```bash
catkin_make --pkg neupan_ros -j2 -l2
source devel/setup.bash
```

选择其中一个启动，两个入口使用同一车辆和场景：

```bash
roslaunch neupan_ros planner_comparison.launch planner:=neupan scenario:=static rviz:=true
roslaunch neupan_ros planner_comparison.launch planner:=teb scenario:=static rviz:=true
```

在 RViz 中点击 `2D Nav Goal` 下发目标；`scenario:=crossing` 为固定障碍绕行与会主动让行的移动障碍交互，`scenario:=crossing_stress` 保留原来不避让车辆的强制横穿压测。两者用途不同，自动对比和录像方法见 [规划器对比说明](docs/planner_comparison.md)。

## 室外返程测试

使用 Gazebo 射线雷达、生产点云转扫描节点、OSM A*、[室外配置](../lxw_car/config/neupan_config_outdoor.yaml)、模型 `acker_robot_default_6` 与 NeuPAN 求解器。不启动实车驱动、串口、MID360、GNSS 或 GPU 接收。

四组自动测试并录制 rosbag：

```bash
rosrun neupan_ros outdoor_simulation.py run --scenario all --record-bag
```

只看宽路往返，打开 Gazebo 与 RViz：

```bash
rosrun neupan_ros outdoor_simulation.py run --scenario wide --gui --rviz --record-bag
```

默认使用共享 C++ Hybrid A* 的室外前进连接模式，独立进程限时搜索，失败保持停车。旧 Dubins 对照加 `--connection-planner dubins`；它不是室内模式，也不打开旧整段栅格规划分支。

入口自动创建独立 ROS master 和 Gazebo master，测试结束后关闭；控制与 TF 使用 `/outdoor_sim/*`，无需配置网卡 IP。不要把室外 launch 直接加载到正在控制实车的 master，避免 `/clock` 与 `/use_sim_time` 影响实车。

| 场景 | 路宽 | 过程与通过条件 |
|---|---|---|
| `wide` | 12 m | 前进约 10 m，再掉头返回；完成连接、到达目标、车体未越界 |
| `narrow` | 3 m | 同样下发返程目标；确认无安全连接，持续零速度至少 2 s、无漂移 |
| `unknown` | 12 m | 到第一目标后将回波置为 `inf`；不能把未知区域当成空路，拒绝掉头并停车 |
| `obstacle` | 12 m | 到第一目标后在左右转弯区加入实体障碍；道路仍宽，必须因雷达障碍证据拒绝掉头并停车 |

后三组必须因 `no_safe_forward_connection` 停车，并通过 `admission` 分别确认道路范围、车身未观测、已观测障碍的拒绝依据；TF 失配或传感器未就绪不能冒充通过。雷达按原始采集时刻匹配 TF，不重写时间戳。到达阈值沿用生产配置的 0.5 m，并检查 Gazebo 车体与虚拟里程计最大位置差不超过 0.25 m。

2026-10-16 起，脚本只读实车 launch 与 FAST-LIO 配置，将当前雷达/IMU 安装关系、三维 `blind=0.5 m` 和扫描参数写入每次结果清单。虚拟雷达在 `(0.1,0,0.35) m` 采样，经 LiDAR→IMU 点云变换，再交给真实 `pointcloud_to_laserscan_node` 按 `base_link`、`range_min=0.4 m`、高度 `[0.2,0.8] m` 投影；不再直接把较小盲区的理想 LaserScan 交给规划器。原实车雷达配置没有修改。

掉头检查区分实际车身与额外 `0.12 m` 缓冲区：实际车身未来进入的车外区域必须有有效观测；已知障碍仍检查缓冲区。仅位于近场过滤圆内的额外缓冲采样点可以保持未知，不将其认作空地，状态明确记录 `unverified_near_margin_points`、`margin_fully_observed=false`。这不代表完整 12 cm 余量已经得到观测验证；车身本体进入盲区、外侧未知区、碰到障碍或缺少新鲜 TF 仍拒绝。采集时刻 TF 延迟会短队列重试，保留上一份仍在 0.5 s 有效期内的对齐扫描，不用最新 TF 代替。

室外车辆以后轴为参考点，轴距 0.67 m、规划外形 0.95×0.85 m、最小转弯半径 1.85 m；`base_link` 离地 0.20 m。OSM 路宽与 [worlds/outdoor_roads.world](worlds/outdoor_roads.world) 的物理边界一致，路网为 [maps/outdoor_test.osm](maps/outdoor_test.osm)。

### 结果

共享 Hybrid A*：[20261016_113623 四场景报告](../../../../videos/outdoor_simulation/20261016_113623/summary.md)。四组均通过，宽路返程误差约 0.488 m（到达阈值 0.5 m），搜索约 26 ms；后三组拒绝后持续零速度，未启动实车。旧 Dubins 对照保留在 [20261016_092222](../../../../videos/outdoor_simulation/20261016_092222/summary.md)。这些场景不代表实车最小掉头宽度。

```text
/home/nuc/lxw/neupan_ws/videos/outdoor_simulation/日期_时间/
├── summary.md / summary.json
├── wide/
├── narrow/
├── unknown/
└── obstacle/
```

每个场景保存 `overview.mp4`（H.264 俯视轨迹）、`navigation.jsonl`、`result.json`、`console.log`、`run_manifest.json`、自动生成的 `scan_parameters.json` 和 `ros_logs/`；加 `--record-bag` 后保存 `simulation.bag`。异常时 JSON/NPZ 也放在该场景目录。`--output-dir` 可覆盖保存位置。

视频不是摄像头画面：蓝色为 OSM 路线、绿色为实际接收参考线、粉色为局部规划、黄色为行驶轨迹。定位采用仿真真值，虚拟 GPS 仅提供接口；不验证真实 RTK、FAST-LIO、坡道、打滑或 HybridNets 靠右效果。

### 两种掉头方案的窄路对比

同一个入口可从完全相同的静止车姿 `(10,0,0)`，对照原 Dubins 曲线与共享 Hybrid A*，不先靠边、不倒车。每种宽度调用完整 OSM→连接段→NeuPAN→Gazebo 链路，以实际完成返程且车身不越界为准，不仅检查能否生成路径。

```bash
rosrun neupan_ros outdoor_simulation.py compare \
  --widths 10 9 8 7 6 5 --width-step 0.25 --repeats 2
```

初筛后自动在完成/未完成之间细化到 0.25 m，再复测临界宽度。Hybrid A* 沿用生产 1 s 搜索预算；搜索耗尽只能说明当前算法/预算未找到路线，不是物理上绝对不能掉头。若临界结果重复不一致，报告不应当作稳定最小宽度。

结果在 `videos/outdoor_simulation/日期_时间_width_comparison/`：保留 `comparison.md`、全部数值 `comparison.json`、临界/异常视频与轨迹日志。若最窄成功宽度复测失败，自动向更宽的已完成案例验证，区分偶发成功和重复通过。结束后删除本轮 `_work/` 中生成的场景/地图、内部 ROS 日志和重复录像，并按专属日志目录清理残留仿真子进程；不清理历史实车 rosbag 或日志。该命令不改实车、室内或规划器配置。

2026-10-16 [21 次窄路对照](../../../../videos/outdoor_simulation/20261016_142251_width_comparison/comparison.md)：两者重复全部完成的最窄已测宽度均为 9 m（2/2）。新版 8.75 m 为 1/2 完成，另一次包络越界；8.5 m 两次均越界，8 m 也未走完，不能将离线路径生成成功当作实车能力提升。当前应先处理紧弯跟踪余量；这不是实车安全路宽结论。

## 原室内 / 局部仿真

保持原启动方式，在 RViz 使用 `2D Nav Goal`：

```bash
roslaunch neupan_ros lxw_car_simulation.launch
```

自动发布演示目标：

```bash
roslaunch neupan_ros lxw_car_simulation.launch publish_demo_goal:=true
```

该入口不走室外 OSM A*，使用 [config/neupan_config_sim.yaml](config/neupan_config_sim.yaml)、模型 `acker_robot_default_2` 和局部车辆模型；规划宽度为 0.75 m。默认启用自适应避障参数，不要与室外 0.85 m 规划外形混用。原组件入口 `neupan_lxwcar_gazebo.launch` 仍可直接启动。

2026-10-19 仿真修正：`base_footprint` 改为后轴参考，`base_link` 位于其前方0.335 m，与 NeuPAN Ackermann 外形一致；原局部规划速度上限收敛到参考速度0.5 m/s，并扩大参考线重捕获范围。二维射线高于车体，默认关闭原先以后轴为中心的对称车体自过滤，避免丢弃近场真实障碍。共用底盘增加非有限指令拒绝、时钟复位及精确圆弧积分，实车配置未修改。

原局部入口的默认场景为 `worlds/lxw_car_obstacles.world`，不依赖 RVO。可用 `cad_visual:=false` 显示基础几何外观，用 `show_goal_marker:=false` 关闭目标标记。

### NeuPAN / TEB 对比

对比工具已归入本目录，使用相同车辆、10 Hz控制、扫描、参考线和可重复动态障碍对比两者，详见 [规划器对比说明](docs/planner_comparison.md)：

```bash
rosrun neupan_ros planner_comparison.py run --scenario dynamics --teb-profile field --repeats 2 --record-gazebo
```

NeuPAN 默认使用原局部仿真的自适应调参（`--neupan-profile adaptive`），历史固定参数实验可用 `--neupan-profile fixed` 复现。

结果在 `videos/planner_comparison/日期_时间/`，包括真实 Gazebo 录屏、轨迹、间距和性能报告。TEB可单独用 `--planner teb --scenario crossing --repeats 1 --gui` 运行。默认使用用户确认的实车 TEB 参数的仿真适配副本，`--teb-profile baseline` 可比较原仿真参数组；不修改 `src/teb_test` 的实车配置。

### 动态障碍物（可选）

先确保 `rvo_ros`、`libmulti_obs.so` 及场景引用的模型可用：

```bash
roslaunch neupan_ros lxw_car_simulation.launch \
  dynamic_obstacles:=true \
  world_name:=$(rospack find neupan_ros)/example/lxw_car_simulation/worlds/gazebo_world_complex_20.world
```

默认障碍物最大速度 0.5 m/s，可用 `obstacle_max_speed:=0.8` 调整。随机目标范围为 x=[1.5,9.0]、y=[-4.0,4.0]，RVO 时间窗 2 s；随机模式连续 4 s 未移动 0.20 m 时重设目标。目标客户端最多等待服务 15 s，启动后应确认目标服务调用成功。过大的时间窗或过窄目标范围可能让人群停滞，应以模型实际位移判断运动，而非仅看配置速度。

## 共用模型与限制

- CAD 网格 `meshes/lxwcar_cad.stl` 来自毫米单位的 STEP，按 0.001 缩放，约 0.938×0.745×0.359 m，仅作外观；简化碰撞体与传感器仍独立生效。STL 不保留 SolidWorks 颜色，Gazebo 使用灰色材质。
- 实车 MID360 是 3D 点云；室外这里仅生成安装高度处的二维射线切片并转成点云，复现实车外参/过滤/投影链，不模拟真实扫描花纹、遮挡、噪声、低矮障碍或完整 FAST-LIO。室内旧虚拟传感器参数单独保留，不作为实车标定。
- 共用虚拟底盘积分 Ackermann 模型，发布里程计、TF 和轮胎/转向关节状态，再通过 `/gazebo/set_model_state` 更新车体。它不是电机或轮胎动力学插件；默认指令超时 0.5 s 自动停止（`cmd_timeout`）。
- 仿真通过不代表实车验收。局部仿真的后轴参考与限速修正见上；室外配置、实车参数与共享 Hybrid A* 本轮未修改。

最新完整对比与视频入口：[2026-10-19 自适应 NeuPAN / TEB](../../../../videos/planner_comparison/20261019_adaptive_full_comparison/presentation/README.md)。旧失败结果已清理，本轮失败及审计异常完整保留。

## 室外道路避障与真实 OSM 场景

新增 [室外道路仿真说明](docs/outdoor_road_simulation.md)：读取当前室外参数与模型 `_7`，独立运行校园道路静态绕障/横穿挑战，支持真实 Gazebo 1080p 录屏。静态测试通过，横穿仍有包络重叠，尚未验收。还可从实车现有 `map.osm` 生成保留路网/建筑轮廓的校区场景；楼高、路宽缺失处明确标为假设，真实路网闭环导航尚未验证。

2026-10-24 [最新室外效果视频与报告](../../../../videos/outdoor_simulation/20261024_adaptive_road_recording/README.md)：NeuPAN＋扫描运动估计＋预测让行保护（仅仿真），静态1/1、横穿2/2到达。原生接入与纯速度接入仍有失败，不能将结果归为原生规划器单独通过；实车及求解器未改。


园区5动态障碍物、原生室外参数录像（关闭速度输入和额外让行保护）：

```bash
rosrun neupan_ros outdoor_road_simulation.py run --scenario campus --profile outdoor --motion-estimation off --predictive-yield off --record-gazebo
```

开关说明见 [室外道路仿真](docs/outdoor_road_simulation.md#园区多动态障碍物与显式开关2026-10-24)。


园区回线修复（速度估计与预测让行关闭，启用扫描时间对齐和求解双初值）：

```bash
rosrun neupan_ros outdoor_road_simulation.py run --scenario campus --profile sim_campus --motion-estimation off --predictive-yield off --record-gazebo
```

实际参数见每次输出的 `effective_planner.json`，仿真不修改实车文件。
