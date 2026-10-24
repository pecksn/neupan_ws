# 室外道路 NeuPAN 避障仿真

## 原始接入检查（2026-10-23）

1. 原返程仿真固定使用 `_6`，而当前 `full_system_run.launch` 的室外默认分支已使用 `_7`。现由仿真脚本只读解析生产入口，记录配置与模型哈希；原返程入口也同步到当前模型。历史结果仍对应历史版本。
2. 原返程测试不能代替正常道路静态绕障与动态横穿测试。新增两个独立道路场景。
3. 当前室外默认 `enable_adaptive_adjust=false`，与此前室内对比的 adaptive 模式不同。新仿真按室外原值运行，没有为了通过测试开启自适应。
4. 当前参数在横穿挑战中仍未通过：车辆开始侧向绕行后，NeuPAN触发原生停止，固定时序障碍继续移动并与车体包络重叠。初次测试还捕获一次 `ECOS` 求解器异常；不能把异常、运动学接触和算法普遍能力混为同一个原因。

所有控制改动均在仿真入口；生产配置、模型权重、NeuPAN求解器、实车节点均未修改。

## 启动室外避障

终端输入 `Neupan`、按 `e` 后：

```bash
conda activate lxw
source /opt/ros/noetic/setup.bash
cd /home/nuc/lxw/neupan_ws
source devel/setup.bash

# 在自己的 Gazebo/RViz 窗口观察静态绕障
rosrun neupan_ros outdoor_road_simulation.py run --scenario static --gui --rviz

# 静态绕障与固定时序横穿；独立虚拟显示录制真实 Gazebo 1080p 窗口
rosrun neupan_ros outdoor_road_simulation.py run --scenario all --profile sim_motion_yield --record-gazebo
```

每个场景自动建立独立 ROS/Gazebo master。不要用普通实车全系统入口代替这里的命令。若尚无 `devel/lib/liblxw_sim_pose.so`，先 `catkin_make --pkg neupan_ros -j2 -l2` 并重新 source。

`--scenario crossing --profile outdoor` 是原参数接入未通过的非合作横穿挑战。障碍固定在 x=13 m，起始 y=3.3 m，第9秒以0.55 m/s横穿到-3.3 m，不根据车辆行为让路。规划器只接收扫描，没有障碍真值速度。静态场景在 `(8,0)` 放置半径0.4 m圆柱，目标为 `(18,0)`。

生成校园道路包含8米道路、路沿、人行道、斑马线、建筑、树木和停车。建筑/车辆外观是简化几何，属于可重复测试环境，不是实景重建。停车、到达及包络检查均基于实际 Gazebo 状态；到达需原生完成、目标误差≤0.5 m、航向误差≤0.35 rad、速度≤0.1 m/s稳定0.5秒。包络重叠允许2 cm数值容差，结果记录实际有符号净距；这不是安全余量。

底盘使用同一仿真位姿插件，Ackermann轴距0.67 m；规划包络0.95×0.85 m。当前室外参考速度0.8 m/s、上限1.5 m/s，固定调参，模型 `_7`。不是此前限速0.5 m/s的TEB对比配置。

运行结果写入 `videos/outdoor_simulation/日期_时间_road_avoidance/`：

- `static/`、`crossing/`：独立场景。
- `gazebo.mp4`：实际Gazebo窗口录屏（加录制参数后生成），1920×1080、15fps，按墙钟录制。
- `trajectory_1080p.mp4`：实际Gazebo轨迹俯视回放，1920×1080、10fps，按仿真秒播放；不是相机录像。
- `result.json`、`trajectory.jsonl`、`telemetry.bag`、`transport_audit.json`：实际到达、几何净距、消息连续性及独立终点检查。
- `inputs/`、`manifest.json`：生成世界、OSM、参数副本及源文件哈希。运行时只读生产配置。

当前例程使用0.5倍目标墙钟速率，不能据此证明实车实时性能。定位采用真值；雷达是水平ray切片，经LiDAR/IMU转换与生产 `pointcloud_to_laserscan`，保留盲区与过滤参数，但不模拟完整MID360采样、3D遮挡、RTK、FAST-LIO误差或相机路沿识别。虚拟底盘不是轮胎/完整接触动力学仿真。

## 扫描运动接入（2026-10-24，仅仿真）

`--profile outdoor` 保留原生ROS接入与实车参数；`--profile sim_motion` 保留相同室外参数、速度及 `_7` 权重，切换仿真专用 `outdoor_motion_controller.py`。不改实车节点和求解器。

当前本机NeuPAN库已有 `forward(state, points, velocities)`，但原ROS节点只传前两项。新入口以采集时刻TF将扫描转换到固定map坐标，通过短时紧凑点簇关联、回归拟合及点表面一致性估计速度，送入库已有预测接口；同时补偿从采集到求解的时间。没有订阅Gazebo障碍真值，也没有读取障碍时序给规划器。真值只供独立评估。原生路径处理、跟踪保护及扫描超时保护继续使用。

100Hz时钟可能略晚于扫描时间戳：扫描先进入有界等待，等采集时刻TF可用再处理，不重写时间戳、不取消0.5秒过期保护。扫描视野边界或停放车辆可造成“质心滑动”，因此还检查点表面静止/运动假设，并拒绝仅沿平面切向滑动的运动推断。

`--profile sim_motion_yield` 在扫描运动接入之外增加独立仿真让行监督：用已观测紧凑点簇的匀速外推及车体沿当前航向继续行驶的5秒包络预测，风险净距小于0.3米时暂时置零动作，连续0.5秒无预测冲突后恢复。预测物体半径由观测点簇加余量估计，不读取实际圆柱半径、轨迹或Gazebo真值。原生规划器仍负责路线和绕障，但让行动作来自新增监督层，不能归功于原生NeuPAN单独完成。画面和日志分别标明保护状态。

仅速度接入在一次复测中通过、另一次失败，不作为稳定改善方案；最终演示明确使用“NeuPAN＋扫描预测让行保护”。

这是实验性扫描运动估计与直行走廊保护：紧凑点簇过少、遮挡/融合、极慢运动（<0.25 m/s）和复杂交通仍可能估计不足或误判；不能直接移植实车或宣称普遍动态安全。

`sim_adaptive` 为本次调参试验，曾到达但途中净距接近零，**不推荐作为改善方案**。展示视频会明确标注到达但余量不足，不将2cm数值容差当作安全距离；额外检查有限采样净距至少0.1米。

最新分类结果和带中文状态的视频入口：`videos/outdoor_simulation/20261024_adaptive_road_recording/README.md`。原版失败和调参尝试会在报告中分别说明，最终版本所有复测均保留。

## 本地真实校区 OSM 转场景

已经发现生产地图 `/home/nuc/lxw/ros_maps/map.osm`，可离线转换，不下载模型：

```bash
rosrun neupan_ros outdoor_osm_world.py \
  --osm /home/nuc/lxw/ros_maps/map.osm \
  --output-dir /home/nuc/lxw/neupan_ws/videos/outdoor_simulation/my_campus_scene
```

默认以生产地图原点 `(23.0398,113.39002)` 转为局部米制坐标，选取半径250米内路段/建筑。道路保留OSM折线，建筑按OSM轮廓挤出。OSM有路宽/高度时使用原值；无路宽时按已知车道数×3.2 m或默认6 m，无楼高时按层数×3 m或默认9 m，逐项记录到 `scene_manifest.json`。地面为平面，没有真实纹理和实测高程，暂不处理relation多面建筑。

这是**真实平面地图几何的场景预览**，不是照片级数字孪生；尚未将真实校区路网作为闭环避障验收结果。要进一步接近实景，需要标定后的点云/建筑模型、实测路宽与高程，必要时加真实纹理；OSM不能补出未记录的路沿和障碍。

预览生成世界（使用空闲的独立端口）：

```bash
ROS_MASTER_URI=http://127.0.0.1:11329 \
GAZEBO_MASTER_URI=http://127.0.0.1:11359 \
roslaunch -p 11329 gazebo_ros empty_world.launch \
  world_name:=/home/nuc/lxw/neupan_ws/videos/outdoor_simulation/my_campus_scene/campus.world \
  gui:=true use_sim_time:=true
```

## 本次结果入口

- [最新室外视频与结果](../../../../../videos/outdoor_simulation/20261024_adaptive_road_recording/README.md)
- [校区真实OSM几何场景](../../../../../videos/outdoor_simulation/20261023_gdut_osm_campus/README.md)

上述2026-10-23结果对应原版接入。2026-10-24预测让行保护版本静态1/1、横穿2/2到达，横穿最小采样净距约1.14米；纯运动接入仍有失败。复测见最新分类目录，有限仿真通过不代表实车验收。旧室内TEB对比的6/6不能套用到现在的室外参数。


## 园区多动态障碍物与显式开关（2026-10-24）

两个命令行开关优先于 `--profile`，每次运行都会把最终开关状态、控制器类型、障碍物轨迹保存到 `manifest.json` 和 `inputs/control.yaml`。

```bash
# 两项辅助功能全部关闭：原生 NeuPAN ROS 节点，室外原参数和模型
rosrun neupan_ros outdoor_road_simulation.py run --scenario campus --profile outdoor --motion-estimation off --predictive-yield off --record-gazebo

# 仅向 NeuPAN 加入扫描估计的障碍物速度，不加提前停车保护
rosrun neupan_ros outdoor_road_simulation.py run --scenario campus --profile outdoor --motion-estimation on --predictive-yield off --record-gazebo

# 两项全部开启
rosrun neupan_ros outdoor_road_simulation.py run --scenario campus --profile outdoor --motion-estimation on --predictive-yield on --record-gazebo
```

- `--motion-estimation on/off`：控制向 NeuPAN 输入估计速度及扫描点运动补偿。
- `--predictive-yield on/off`：控制独立预测冲突后的停车保护；关闭后原生 NeuPAN 仍可能自己停车，不承诺不停顿。
- 仅打开让行保护时，保护层仍需内部扫描运动估计，但不向 NeuPAN 输入估计速度。两项都关闭时直接使用 `neupan_node.py`，不启动运动适配器。
- 不传开关时保持 profile 的原行为，不改变旧命令的含义。

`campus` 场景为生成的园区道路，包含5个半径0.3米的简化动态行人：3个分时双向横穿（0.85～1.0m/s），2个沿道路双向移动（0.7、0.9m/s）。所有轨迹都只依赖仿真时间，终点停止，不读取车辆位置，不避让车辆。沿道路逆向行人的路线位于停车区内侧，避免穿过停放车辆。每个行人的位置都写入 `trajectory.jsonl` 的 `obstacles`；几何检查和 rosbag 独立审计覆盖全部5个障碍物，视频小地图也显示全部障碍物。


### 本轮原生园区测试结果

历史原生失败诊断已保存在 `videos/outdoor_simulation/20261024_campus_repair/checks/native_baseline/`；最新修复视频位于 `videos/outdoor_simulation/20261024_campus_repair/presentation/neupan_outdoor_gazebo_1080p.mp4`。实际运行的是 `neupan_node.py`；5个障碍物的位移及速度均经实际模型状态核验。

本轮在约19.57秒触发已有的室外路径跟踪保护 `route_tracking_lost`（横向偏离约1.76米，阈值1.5米），随后未恢复，75秒超时。它不是新增的预测让行，也没有原生求解器 `planner_stop=true` 记录。独立 rosbag 最小采样净距约0.089米，没有采样到包络重叠，但不足0.1米，不认定为安全完成。完整录像和诊断均保留。本次按用户要求关闭两项辅助功能，保留生产原有的扫描及路径跟踪保护。


## 园区回线修复（2026-10-24）

`sim_campus` 保留当前室外全部规划参数、模型和保护阈值。速度输入、运动补偿和预测让行保护均关闭；只在仿真适配器中：

1. 按扫描采样时刻查询 TF；两项运动功能都关闭时不运行速度估计器。
2. 比较沿用上一步控制序列与中性控制序列两个求解初值，仍由原 NeuPAN 求解器求解。优先排除静态扫描几何预测穿透的候选，再比较跟踪、速度和余量代价；完整恢复选中候选的进度、控制、停止状态和距离缓存。没有绕过原生扫描或路径跟踪保护，不直接改写转角或强制放行。
3. 修复仿真 launch 内联默认参数覆盖 `control.yaml` 的问题。有配置文件时由配置文件提供控制参数；地图原点仍显式提供。记录 `effective_planner.json`，以实际对象中的权重和步数核验，而不只展示输入 YAML。

同一份失败位置/扫描的离线探针表明，负转向初值会得到预测横向偏离约3.34米的劣解；中性初值得到约0.59米、正确回线的解。该证据说明本场景存在求解分支依赖，不能据此判断实车也一定有故障。

```bash
rosrun neupan_ros outdoor_road_simulation.py run --scenario campus --profile sim_campus --motion-estimation off --predictive-yield off --record-gazebo
```

这是带求解初始化修复的仿真模式，视频显式标注“初值修复：开”。`--profile outdoor` 保留原节点基线；`--profile sim_aligned` 只修复扫描对齐，不打开双初值。实车文件和本地 NeuPAN 库没有修改。

本轮曾试验提高回线权重和迭代次数，但 launch 默认值覆盖使生成配置未实际生效，不能把那些试验归因于其请求参数。最终撤回全部规划参数覆盖，保留同一5障碍物轨迹。未通过诊断的紧凑结果将保存在本轮 `checks/`，不混入最终通过统计。


最终录像的同版本两轮均到达（24.32秒、23.72秒），独立 rosbag 最小采样净距0.1473米、0.1470米；运行1秒后且目标0.8米以外没有低于0.05m/s的停车，原有路径保护没有触发。每轮实际 `pan.iter_num=2`、`iter_threshold=0.1`，全部室外权重与模型不变，35项相关测试通过。视频约52秒，两轮完整保留。结果属于仿真求解初值扩展，不是实车实测或完全原样的原生节点表现。
