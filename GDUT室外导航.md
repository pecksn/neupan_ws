# GDUT 小车室外导航

## 目录

- [一、日常运行速查](#一日常运行速查)
  - [编译与环境准备](#11-编译与环境准备)
  - [三种室外导航模式](#12-三种室外导航模式)
  - [Rosbag 一键录制](#13-rosbag-一键录制)
  - [Rosbag 离线路缘测试](#14-rosbag-离线路缘测试)
- [二、Jetson、Docker 与基础环境](#二jetsondocker-与基础环境)
- [三、Livox MID360 雷达](#三livox-mid360-雷达)
- [四、端到端模型概况](#四端到端模型概况)
- [五、NeuPAN 部署、参数与仿真](#五neupan-部署参数与仿真)
- [六、TEB 与其他全局算法](#六teb-与其他全局算法)
- [七、URDF、Gazebo 与 Docker 迁移](#七urdfgazebo-与-docker-迁移)
- [八、地图、SLAM 与全局规划](#八地图slam-与全局规划)
- [九、点对点导航](#九点对点导航)
- [十、跟踪算法与 Nav2](#十跟踪算法与-nav2)
- [十一、RTK/CORS](#十一rtkcors)
  - [G90 位置与双天线定向状态](#112-g90-位置与双天线定向状态2026-10-14-更新)
- [十二、道路视觉与相机](#十二道路视觉与相机)
  - [MID360 激光雷达路缘检测](#121-mid360-激光雷达路缘检测)
  - [USB Camera3 与远程图像](#122-usb-camera3远程图像与-devvideo0)
  - [相机内参标定](#123-相机内参标定)
  - [MID360-Camera 外参标定](#124-mid360-camera-外参标定)
  - [FAST-Calib](#125-fast-calib-适配)
  - [HybridNets](#126-hybridnets-可行驶区域与车道线检测已实现实时接收)
  - [GPU板→NUC](#127-gpu-板--nuc-实时视觉传输已实现)
  - [Vision + LiDAR 融合](#128-vision--lidar-道路约束融合待完成)
  - [历史视觉/相机方案](#129-历史视觉与相机方案)
- [十三、VLM 与世界模型](#十三vlm-与世界模型)

# 一、日常运行速查

## 1.1 编译与环境准备

nuc15 已安装原视频接收开机自启动（共享 ROS Master + UDP 5000 → `/camera/remote_image_raw`、`/dev/video0`），不启动整车或 JSON；JSON 仍由全系统启动。登录桌面后会显示接收状态终端，无视频弹窗。宿主机可用 `journalctl -u neupan-remote-video.service -n 20 -f` 查看实际接收 FPS。

nuc15 接口、固定串口别名和迁移检查见 [nuc15设备接入.md](nuc15设备接入.md)。主机目录为 `/home/nuc15/lxw/neupan_ws`，容器内仍映射为 `/home/nuc/lxw/neupan_ws`。底盘默认串口改为 `/dev/robot_base`，RTK 保持 `/dev/wheeltec_gnss`。

NUC 主机输入 `Neupan`、按 `e` 进入容器，以下命令都在 ROS1 Noetic 环境中执行。日常启动无需重新编译；可直接使用 [核心启动指令.md](核心启动指令.md) 中的整段命令。

```bash
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
source /home/nuc/lxw/neupan_ws/devel/setup.bash
```

每个新终端至少执行：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
```

## 1.2 三种室外导航模式

### OSM A* + 全局路径直接跟踪（不带局部避障）

适合先验证 GPS、地图坐标、车辆航向和底盘控制：

```bash
roslaunch neupan_ros outdoor_global_only_test.launch \
  direct_tracker_yaw_offset_deg:=0 \
  direct_tracker_projected_goal_stop_tolerance:=1.0
```

### OSM A* + NeuPAN 局部规划与避障

```bash
roslaunch neupan_ros full_system_run.launch outdoor_mode:=true
```

云端经纬度路径 `/osm_global_path_ll` 默认启用几何简化：直线仅保留首尾点，转弯保留必要形状点，允许约 0.10 米的几何偏差；本地 `/osm_global_path` 的规划/跟踪密度不变。话题名和 `{"points":[{"latitude":...,"longitude":...}]}` 格式不变，云端点数不再与本地路径一一对应。需要恢复发送全部路径点时追加 `cloud_path_simplify:=false`；用 `cloud_path_simplify_tolerance:=0.10` 调整容差，容差为 0 也禁用过滤。规划日志 `Cloud path published: N -> M points` 可查看压缩前后点数；需要重新编译 `osm_map_handler` 并重启规划器后生效。

当前全系统的两个室外 NeuPAN 分支均使用 `example/lxw_car/model/acker_robot_default_6/model_5000.pth`。`neupan_config_outdoor.yaml` 的车体长 `0.95 m`、宽 `0.85 m`、轴距 `0.67 m`，与新模型训练几何一致；参考速度 `0.8 m/s`。更换模型时必须同步核对车体几何，不能仅替换 checkpoint。旧模型 `_5`（训练宽度 `0.60 m`）保留用于对比。

该启动文件默认接收 GPU 板原视频和 HybridNets JSON，同时发布 `/hybridnets/debug_image`，但不弹出摄像头画面窗口。需要临时查看叠加画面时：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true \
  show_hybridnets_overlay:=true
```

临时不接收 GPU 板数据时增加 `enable_hybridnets_receiver:=false`。全系统已经启动接收器时，不要再单独启动 `receiver.launch`，否则 UDP 端口会冲突。

仅关闭调试叠加图增加 `enable_hybridnets_overlay:=false`，原视频、虚拟摄像头和 JSON 继续接收；`show_hybridnets_overlay:=false` 只是不弹窗，不会停止绘图。关闭绘图后不产生 `/hybridnets/debug_image`，HybridNets 专用录包配置会告警缺少该必录话题；原图、JSON 和雷达仍可录制。

### OSM A* + TEB

```bash
roslaunch teb_test outdoor_teb_test.launch
```

对应关系：

| 启动文件 | 全局规划 | 局部控制/避障 | 主要用途 |
|---|---|---|---|
| `outdoor_global_only_test.launch` | OSM A* | `global_path_tracker`，无局部避障 | 定位、航向、路径跟踪验证 |
| `full_system_run.launch outdoor_mode:=true` | OSM A* | NeuPAN | 完整室外自主导航 |
| `outdoor_teb_test.launch` | OSM A* | TEB | TEB 对照测试 |

## 1.3 Rosbag 一键录制

通用录包包 `src/vehicle_bag_recorder` 用 YAML 配置话题，由 `rosbag record` 保留原始类型和时间戳。默认录制：

- MID360 原始点云和 IMU；
- FAST-LIO 车体/地图点云与里程计，其中 `/cloud_registered_body` 是路缘离线复现的必录点云；
- GPS、航向、TF 和定位保护状态；
- 原始 OSM 路径、路缘修正路径和路缘检测结果；
- `/scan`、NeuPAN/直跟控制指令和目标点。

启动：

```bash
sudo apt install python3-yaml
python3 -c "import yaml; print('PyYAML OK')"
roslaunch vehicle_bag_recorder record.launch
```

`python3-yaml` 只需安装一次；若报 `ModuleNotFoundError: yaml`，检查 ROS1 容器的 Python 环境。

录制器检查 `/cloud_registered_body`、`/tf`、GPS、航向和本地相机话题；缺失时每 5 秒告警，结束时列出从未出现的必录话题。启动成功不等于数据已录到。

也可以直接运行 Python：

```bash
rosrun vehicle_bag_recorder bag_recorder.py \
  --config $(rospack find vehicle_bag_recorder)/config/vehicle_data.yaml
```

临时指定输出目录和测试名称：

```bash
roslaunch vehicle_bag_recorder record.launch \
  output_directory:=/media/usb/rosbags \
  bag_prefix:=gdut_navigation_test
```

默认保存目录：

```text
/home/nuc/lxw/neupan_ws/rosbags/
```

默认文件名类似 `vehicle_data_20260908_091500_0.bag`；可用 `bag_prefix` 临时改名。

默认 LZ4 压缩、每 2048 MB 分包、剩余空间低于 8 GB 时停止，并生成 JSON 清单。按 `Ctrl+C` 后等“录制结束”，再断电或拔盘。

### 双板 HybridNets 专用录制

GPU 板作为正式相机源时，使用专用配置，记录远程原视频、叠加调试图、内参、推理 JSON、有效状态、推理频率、网络时延、Livox/FAST-LIO 点云和 TF：

```bash
roslaunch vehicle_bag_recorder record.launch \
  config_file:=$(rospack find vehicle_bag_recorder)/config/hybridnets_data.yaml
```

摄像头保持插在 GPU 板；NUC 解码图像为 `/camera/remote_image_raw`，内参为 `/camera/remote_camera_info`。每次录制新建目录并按 2048 MB 分包：

```text
rosbags/rosbagYYYYMMDD_HHMMSS/
├── hybridnets_vehicle_data_YYYYMMDD_HHMMSS.json
├── hybridnets_vehicle_data_YYYYMMDD_HHMMSS_0.bag
└── hybridnets_vehicle_data_YYYYMMDD_HHMMSS_1.bag
```

录制前检查：

```bash
rostopic hz /camera/remote_image_raw
rostopic echo -n 1 /camera/remote_camera_info
rostopic hz /hybridnets/result_json
rostopic echo /hybridnets/valid
```

结束时按一次 `Ctrl+C`，等待 `.bag.active` 完成写入，再对实际生成的文件检查：

```bash
rosbag info /home/nuc/lxw/neupan_ws/rosbags/rosbagYYYYMMDD_HHMMSS/具体文件.bag
```

确认 `/camera/remote_image_raw`、`/hybridnets/debug_image`、`/hybridnets/result_json`、`/cloud_registered_body` 的消息数均非零。

### 运行卡顿诊断（2026-10-08 更新）

资源采集由独立节点 `robot_base_driver/scripts/system_resource_monitor.py` 完成，全系统随 `enable_system_log` 自动启动；`system_run_logger.py` 负责保存。`neupan_core.py` 仅保留规划耗时/指令间隔记录，不再读取主机资源或生成配置快照。主机进程排名由主机端 `host_process_monitor.py` 采集，独立 ROS 节点读取其快照；两者不在规划计算进程内运行。

全系统默认写入 `tracker_logs/system_run_full_日期_时间.jsonl`；普通及 HybridNets 录包配置均已加入下列诊断话题，重启后生效，录包命令不变。

| 记录 | 排查用途 |
|---|---|
| `/cmd_vel/auto`、`/cmd_vel/selected` | 对比 NeuPAN 原始速度与最终输出 |
| `/vehicle/control_mode/status` | 模式、指令年龄、超时阈值及 `auto_timeout` / `localization_unsafe` 原因 |
| `/neupan/runtime` | 单次求解耗时、TF 查询耗时、成功规划指令更新间隔、速度和转角 |
| `/neupan/failure` | 规划主线程异常类型、完整堆栈、失败阶段及 JSON/NPZ 快照路径 |
| `/neupan/camera_path_guard` | NeuPAN 独立选择摄像头/原路线/停车的状态、心跳年龄和原因 |
| `/neupan_initial_path`、`/neupan_ref_state`、`/neupan_plan` | NeuPAN 实际参考路线、当步参考状态及优化轨迹；三套录包配置均已加入 |
| `/Odometry`、`/odometry/lio_guarded`、`/odometry/local` | 对比原始 LIO、保护后 LIO、EKF 融合定位 |
| `/gps_guard/local_fault_event` | 每次异常触发的位移、消息时间差、接收时间差及两种计算速度 |
| `/gps_guard/status`、`/lio_guard/status`、`/localization_safe`、`/neupan/scan_health` | 定位、扫描与 TF 保护状态 |
| `/gnss/solution_status` | G90 的独立位置解与定向解类型、航向有效性、基线长度及定向卫星数；已接入 JSONL，独立话题尚未加入现有 rosbag 配置 |
| `/motor/status`、`/motor/alarm` | 扭矩、温度和告警；当前 ≥7 N·m 仅告警，不因扭矩超限自动停车 |
| `/neupan/config_snapshot` | 启动配置、模型路径及 SHA256（锁存发布，晚启动录包仍可接收） |
| `/neupan/resources` | 每秒进程 CPU/内存、整机 CPU/内存使用率、I/O 等待率、交换空间及主机 CPU 占用前 10 个进程；CPU 百分比统一按整机为 100% |

JSONL 除 10 Hz 快照外，直接保存原始自动指令、规划耗时、模式变化及定位异常事件，避免短暂异常被快照漏掉；完整配置只保存为独立记录。新增诊断没有改变速度选择、定位跳变判断或停车阈值。

2026-10-15 增加退出诊断：规划主线程出现 Python 异常时，终端和 ROS 日志显示完整 FATAL 堆栈，并与运行日志一起放在 `tracker_logs/`，保存同名 `neupan_failure_日期_时间_*.json`（异常、阶段、路线进度、配置与数值有效性）和 `.npz`（当步车辆状态、障碍点、完整路线、速度预测及可取得的求解器输入）。原异常仍继续抛出，不自动重启。数值快照只在异常时写入；同一份系统运行 JSONL 也保存包含完整异常堆栈的 `neupan_failure_event` 并立即落盘，实际参考路线只在变化时另存完整事件，避免重复写入。

异常补录：独立 `system_run_logger.py` 每秒检查同目录的异常 JSON 文件，ROS 异常事件丢失或规划进程立即退出时仍可补录，日志节点重启后也可恢复已有快照。按快照路径去重，成功写入并 fsync 后才标记已处理，失败会重试；补录事件带 `ingest_source: snapshot_file`，正常话题事件为 `ros`。异常事件不受普通日志大小上限限制。

默认启动方式即可生效；`~diagnostic_log_dir` 可覆盖异常证据的父目录。进程被强制杀死或系统断电时，Python 异常处理不保证执行，此时结合资源日志与 ROS 退出记录判断。三套录包配置还补充了异常话题和实际规划轨迹，HybridNets 配置同时补录 `/scan`。

整机使用率字段：`system_cpu_percent`、`system_cpu_iowait_percent`、`system_memory_percent`；容量字段：`system_memory_total_mb`、`system_memory_used_mb`、`system_memory_available_mb`、`system_swap_used_mb`。CPU 按相邻采样的 `/proc/stat` 差值计算，内存按总量减可用量计算（包含可回收缓存的可用估计），从 Linux `/proc` 读取；当前容器中这些系统计数反映主机整体，不是容器独占额度。`cpu_percent` 和 `rss_mb` 仍仅表示 NeuPAN 进程。

不再记录 `load_average`。`host_top_processes` 保存主机 CPU 占用前 10 个进程的 PID、程序/脚本名称、CPU 百分比、驻留内存、状态和线程数；不保存任意命令参数。`cpu_percent` 统一按整机全部逻辑核心为 100%，即单核口径除以逻辑核心数；原单核口径保留为 `cpu_core_percent`，用 `cpu_percent_basis=whole_machine` 区分旧日志。16 核机器上，原 400% 对应整机 25%。进程前 10 名之和不等于整机总占用，另用 `system_cpu_percent` 查看总占用。主机用户服务 `neupan-host-process-monitor.service` 每秒采集，随 NUC 用户登录启动，容器通过共享目录读取，避免漏掉主机远程桌面等进程。`host_process_snapshot_valid=false` 表示采集器不可用或数据超过 5 秒，不代表没有高占用进程。新增内容重启 NeuPAN 后进入运行 JSONL 和 rosbag。

2026-10-08 晚间日志发现两类卡顿：定位保护触发后保持约 2 秒；定位正常时也出现约 `0.8 → 0 → 0.8 m/s` 的周期切换，疑似规划更新超过默认 `0.5 s` 自动指令超时。后者需用新记录确认，不应直接归因于新模型或夜间光照。

2026-10-09 优化局部运动保护：以消息时间戳在约 `0.2 s` 窗口中计算净位移/航向变化，持续超限约 `0.1 s` 才触发；接收时间只用于新鲜度与诊断，重复、倒序或无效消息不会刷新定位新鲜度。明显瞬时位移超过 `max(0.35 m, 速度上限×消息间隔)`、角度变化超过 `max(15°, 角速度上限×消息间隔)`，或消息时间间隔超过 `0.5 s`，立即触发保护。速度/横向速度/角速度上限及 `2 s` 停车保持不变；GPS 固定解、精度要求和 NeuPAN 指令超时不变。

可调私有参数：`local_motion_window_sec`（0.2）、`local_motion_confirm_sec`（0.1）、`local_motion_max_gap_sec`（0.5）、`local_position_jump_m`（0.35）、`local_yaw_jump_deg`（15）。`/gps_guard/status.local_motion_diagnostics` 保存窗口判断；`/gps_guard/local_fault_event.fault_triggered` 区分实际停车与被忽略的异常时间戳。11 项离线回归测试覆盖正常运动、短间隔厘米级修正、真实位置/角度跳变、持续超限、重复/倒序消息和断流；已有 10 Hz 快照回放未复现原异常，但不能替代全频率 rosbag 和低速实车验证。重启全系统生效。

测试时重点检查：`solver_ms`、`command_interval_ms`、自动指令接收间隔是否超过 `500 ms`，零速度是否对应 `auto_timeout` 或 `localization_unsafe`。`angular.z` 在本车控制话题中是前轮转角（rad），不是车体偏航角速度。

同时录制原视频和调试图会增加 bag 大小。仅在 NUC 本地相机调试时使用 `/camera/image_raw` 和 `/camera/camera_info`；不要与远程相机话题混用。

## 1.4 Rosbag 离线路缘测试

查看 bag 内容：

```bash
rosbag info /home/nuc/lxw/neupan_ws/rosbags/vehicle_data_*.bag
```

终端 1，启动 ROS Master 并启用 bag 时间：

```bash
roscore
```

终端 2，启动路缘检测：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
rosparam set use_sim_time true
roslaunch lidar_curb_refinement road_curb_debug.launch \
  start_path_refine:=true \
  input_path_topic:=/osm_global_path \
  output_path_topic:=/osm_global_path_curb_refined
```

终端 3，回放：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
rosbag play --clock -d 2 /home/nuc/lxw/neupan_ws/rosbags/具体文件.bag
```

暂停、单步和慢速回放：

```bash
# 空格暂停/继续，-r 0.5 表示半速
rosbag play --clock --pause -r 0.5 /path/to/test.bag
```

如果 bag 里已经包含 `/cloud_registered_body`，离线测试路缘时不需要再次运行 FAST-LIO；如果只
录了 `/livox/lidar` 和 `/livox/imu`，则需要先启动 FAST-LIO，再播放原始数据。

回放结束后恢复实时系统：

```bash
rosparam set use_sim_time false
```

---

# 二、Jetson、Docker 与基础环境

本章是首次部署或环境故障时使用的参考记录，已经部署好的实车可直接跳到第三章。


## 2.1 Docker 创建与删除

```bash
wget http://fishros.com/install -O fishros && . fishros
```

ros1,ros1_neupan

[一键安装过程中拉取镜像出现问题 | 鱼香ROS](https://fishros.org.cn/forum/topic/3085/一键安装过程中拉取镜像出现问题/2)

```bash
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<-'EOF'
{
  "registry-mirrors": ["https://ecr650q8.mirror.aliyuncs.com"]
}
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker
```

[鱼香ros docker配置镜像报错：https://registry-1.docker.io/v2/_fishros docker-CSDN博客](https://blog.csdn.net/weixin_42861143/article/details/148491758)

[14个ROS/ROS2版本任选 | 用Docker实现一键安装ROS_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1hY411N7HF/?spm_id_from=333.337.search-card.all.click&vd_source=1aac97640b2d7f2e6ae51b4b1cac54e5)

## 2.2 文件夹权限

若主机用户 `nuc` 无法修改容器录制的视频，在主机执行（仅处理该目录）：

```bash
sudo chown -R nuc:nuc /home/nuc/lxw/neupan_ws/videos/hybridnets_debug
```

## 2.3 代理设置

```bash
export https_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7899
```

## 2.4 Conda（ARM）

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O Miniconda3.sh

bash Miniconda3.sh

source ~/.bashrc
conda --version
```

## 2.5 Docker 无法打开 RViz

### X11 屏幕权限

 主机：

```bash
xhost +local:docker
```

```bash
echo $DISPLAY          # 正常会打印 :0 或 :1
ls /tmp/.X11-unix      # 应该看到 X0 或 X1
xhost                  # 末尾有 “LOCAL:” 或 “SI:localuser:” 字样即可
```

进容器：

```bash
apt update && apt install -y x11-apps
xeyes
```

能看到眼睛 → X11 通了 → 直接 `rviz` 即可。

```bash
echo $DISPLAY
```

- 应该是 `:0` 或 `:1`

- 如果是空的或 `unix`，手动纠正：

  ```bash
  export DISPLAY=:0   # 或者宿主机 echo $DISPLAY 看到啥就写
  ```

```bash
ls -l /tmp/.X11-unix
```

- 应该有 `X0` 或 `X1` 套接字文件。

## 2.6 串口调试工具

**前提是要下载ch340的串口驱动！！！不用在docker里面安装，配置也是docker外就行！！！**

[CH341SER_LINUX.ZIP - 南京沁恒微电子股份有限公司](https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html)

### cutecom

[ubuntu cutecom串口调试工具使用方法（图形界面）-CSDN博客](https://blog.csdn.net/Dontla/article/details/134557362)

```bash
sudo dmesg | grep tty
sudo cutecom
```

![image-20251211164258048](../mdPicture/image-20251211164258048.png)

#### 1. 先确认节点是否存在

```bash
ls -l /dev/ttyCH341USB*
```

如果 **返回“No such file or directory”**，就验证了上面结论。

------

#### 2. 把抢占者干掉（一次性）

```bash
sudo systemctl stop brltty-udev.service
sudo systemctl disable brltty-udev.service
sudo systemctl mask brltty-udev.service   # 彻底屏蔽
```

然后 **重新插拔一次 CH341 转换器**，再查看：

```bash
ls -l /dev/ttyCH341USB*
```

此时应该出现 `/dev/ttyCH341USB0`，cutecom 里也能选到了

### minicom

[Linux下常用的串口助手 —— minicom、putty、cutecom_linux串口助手-CSDN博客](https://blog.csdn.net/Mculover666/article/details/87647810)

**nxsuper：/dev/ttyCH341USB0**

**nuc：/dev/ttyUSB0**

*退出 minicom 的正确姿势是：先按 `Ctrl+A`，松开后按 `X`，然后选 `Yes`)*

```bash
sudo minicom -s
sudo minicom
```


# 三、Livox MID360 雷达

[【3D激光SLAM】Livox-mid-360激光雷达ip配置_mid360-CSDN博客](https://blog.csdn.net/Hahalim/article/details/129414327)

[Livox-SDK/livox_ros_driver2: Livox device driver under Ros(Compatible with ros and ros2), support Lidar HAP and Mid-360.](https://github.com/Livox-SDK/livox_ros_driver2)

[Livox-SDK/Livox-SDK2: Drivers for receiving LiDAR data and controlling lidar, support Lidar HAP and Mid-360.](https://github.com/Livox-SDK/Livox-SDK2)

**主机ip：192.168.1.100**

**雷达ip：192.168.1.183**

**设备码：47MDMCU0030183**

## 3.1 ROS1 驱动部署

### Livox-SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j8
sudo make install
```

### Livox ROS Driver 2

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

#### 编译 Livox ROS Driver 2

```bash
source /opt/ros/noetic/setup.sh
```

在 Livox ROS Driver 2文件夹下

```bash
cd src/livox_ros_driver2/
./build.sh ROS1
```

### 配置主板 IP

```bash
#nx super
sudo ifconfig enP8p1s0 192.168.1.100 netmask 255.255.255.0
#nuc
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
```

### 启动 Livox ROS Driver 2

```bash
source ../../devel/setup.sh
roslaunch livox_ros_driver2 [launch file]
```

```bash
roslaunch livox_ros_driver2 rviz_MID360.launch
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 常见问题

#### 编译工作区的时候这个功能包检测为ros2

##### 方法一

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
```

##### 方法二

```bash
echo 'export ROS_EDITION=ROS1' >> ~/.bashrc
source ~/.bashrc
catkin_make
```

# 四、端到端模型概况


## 4.1 现状

![image-20250910191622464](../mdPicture/image-20250910191622464.png)

## 4.2 分类

![image-20250910191845296](../mdPicture/image-20250910191845296.png)

![image-20250910192318850](../mdPicture/image-20250910192318850.png)

![image-20250910192657481](../mdPicture/image-20250910192657481.png)

## 4.3 模型部署

![image-20251017154624630](../mdPicture/image-20251017154624630.png)

![image-20251017154519864](../mdPicture/image-20251017154519864.png)

![image-20251017154959935](../mdPicture/image-20251017154959935-176546506520551.png)


# 五、NeuPAN 部署、参数与仿真

## 5.1 部署流程


**在Ubuntu22.04中采用ROS1docker运行！！！**

### 工作区创建

```bash
mkdir -p neupan_ws/src
cd neupan_ws/src
git clone https://github.com/hanruihua/neupan_ros
cd.. && catkin_make
cd neupan_ws/src/neupan_ros
bash source_setup.sh && source neupan_ws/devel/setup.sh && rosdep install neupan_ros
```

### livox激光雷达ros功能包部署

（见上）

### neupan ROS功能包部署

[hanruihua/neupan_ros: ROS Wrapper of NeuPAN planner](https://github.com/hanruihua/neupan_ros)]

```bash
git clone https://github.com/hanruihua/neupan_ros
cd ~/neupan_ws && catkin_make
cd ~/neupan_ws/src/neupan_ros
bash source_setup.sh && source /home/nuc/lxw/neupan_ws/devel/setup.sh && rosdep install neupan_ros
```

#### source环境变量问题

```bash
# 重新编译 neupan_ws（确保先加载了 fast_lio 环境）
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/fast_lio_ws/devel/setup.bash

cd /home/nuc/lxw/neupan_ws
rm -rf build devel
catkin_make

# 这样编译出来的 setup.bash 会记住依赖路径
```

### neupan planner部署

#### py310

[hanruihua/NeuPAN: [TRO 2025\] NeuPAN: Direct Point Robot Navigation with End-to-End Model-based Learning.](https://github.com/hanruihua/neupan)

**要在ros1 docker里面进行pip安装！！！**

```bash
python3 -c "import neupan; print('OK')"
```

#### py38

如果你想用python3.8在没有虚拟环境的情况下运行这个项目，可以安装NeuPAN规划器的py38分支：https://github.com/hanruihua/NeuPAN/tree/py38

```bash
#主板上
git clone -b py38 https://github.com/hanruihua/NeuPAN.git /home/hart2204/RoutePlanning/NeuPAN
#nuc上
git clone -b py38 https://github.com/hanruihua/NeuPAN.git /home/nuc/lxw/NeuPAN
cd NeuPAN
python3 -m pip install -e .
#赋予所有指权限
chmod -R 777 /home/hart2204/RoutePlanning/NeuPAN
# 启动Python并尝试导入
python3 -c "import neupan; print('✅ 模块存在'); print(neupan)"
```

#### Ubuntu20.04安装python3.10

[NVIDIA Jetson设备上安装Python 3.10_jetson安装python 3.10-CSDN博客](https://blog.csdn.net/sennsi/article/details/145265652)

合并后的源码安装流程如下。使用 `altinstall`，避免覆盖 Ubuntu/ROS 的系统 Python：

```bash
sudo apt update
sudo apt install -y build-essential checkinstall libreadline-gplv2-dev \
  libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev \
  libc6-dev libbz2-dev libffi-dev zlib1g-dev liblzma-dev
cd /tmp
wget https://www.python.org/ftp/python/3.10.0/Python-3.10.0.tgz
tar xzf Python-3.10.0.tgz && cd Python-3.10.0
./configure --enable-optimizations --with-lzma
make -j$(nproc)
sudo make altinstall
python3.10 --version
python3.10 -c "import lzma; print('LZMA module is available')"
```

缺少 pip 时执行：

```bash
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3.10 get-pip.py
pip3.10 --version
```

不建议修改 ROS 主机的系统 `python3`。确有需要时才使用：

```bash
sudo update-alternatives --install /usr/bin/python3 python3 /usr/local/bin/python3.10 1
sudo update-alternatives --config python3
```

## 设置 NeuPAN 代码权限

```bash
chmod 777 * src/neupan_ros/src/neupan_core.py
chmod 777 * src/neupan_ros/scripts/neupan_node.py

# 然后必须重新 catkin_make
cd /home/nuc/lxw/neupan_ws && catkin_make && source devel/setup.bash
```


## 5.3 NeuPAN 核心参数

### ROS参数

| 参数名                            | 类型  | 默认值       | 描述                                                         |
| --------------------------------- | ----- | ------------ | ------------------------------------------------------------ |
| `~config_file`                    | str   | None         | NeuPAN 规划器配置文件（YAML）的路径（必填）                  |
| `~map_frame`                      | str   | "map"        | 地图坐标系名称                                               |
| `~base_frame`                     | str   | "base_link"  | 机器人基坐标系名称                                           |
| `~lidar_frame`                    | str   | "laser_link" | 激光雷达坐标系名称                                           |
| `~marker_size`                    | float | 0.05         | Rviz 中 DUNE/NRMP 点标记的大小                               |
| `~marker_z`                       | float | 1.0          | Rviz 中机器人标记的 Z 轴高度                                 |
| `~scan_angle_range`               | str   | "-3.14 3.14" | 激光扫描的角度范围（弧度），格式为 "min max"                 |
| `~scan_downsample`                | int   | 1            | 激光扫描数据的下采样率（每隔 N 个点取一个）                  |
| `~scan_range`                     | str   | "0.0, 5.0"   | 激光扫描的距离范围（米），格式为 "min, max"                  |
| `~dune_checkpoint`                | str   | None         | DUNE 模型 checkpoint 文件路径                                |
| `~refresh_initial_path`           | bool  | False        | 是否刷新初始路径（从 goal/waypoints/path 话题更新）          |
| `~flip_angle`                     | bool  | False        | 是否翻转激光扫描的角度数据                                   |
| `~include_initial_path_direction` | bool  | False        | 是否从给定路径中读取方向信息；若为 False，用路径点梯度作为方向 |

| 话题名                 | 消息类型                    | 描述                                         |
| ---------------------- | --------------------------- | -------------------------------------------- |
| `/scan`                | `sensor_msgs/LaserScan`     | 激光扫描数据输入                             |
| `/initial_path`        | `nav_msgs/Path`             | 直接设置初始路径                             |
| `/neupan_goal`         | `geometry_msgs/PoseStamped` | 设置目标点（生成从当前位置到目标的初始路径） |
| `/neupan_waypoints`    | `nav_msgs/Path`             | 设置航点（生成从当前位置到航点的初始路径）   |
| `/neupan_cmd_vel`      | `geometry_msgs/Twist`       | 输出控制速度（线速度 / 角速度）              |
| `/neupan_plan`         | `nav_msgs/Path`             | 输出优化后的路径                             |
| `/neupan_ref_state`    | `nav_msgs/Path`             | 输出当前参考状态                             |
| `/neupan_initial_path` | `nav_msgs/Path`             | 输出初始路径                                 |

### NeuPAN 规划器配置参数（config）

| 模块                     | 参数名                 | 默认值     | 官方含义 / 作用                   | 数据类型 |
| ------------------------ | ---------------------- | ---------- | --------------------------------- | -------- |
| **mpc**                  | receding               | 10         | MPC 预测时域长度（预测步数）      | int      |
|                          | step_time              | 0.2        | 控制步长时间（s），控制周期       | float    |
|                          | ref_speed              | 0.3        | 参考巡航速度（m/s）               | float    |
|                          | device                 | 'cpu'      | 运行设备（仅支持 CPU）            | str      |
|                          | time_print             | False      | 是否打印求解时间                  | bool     |
|                          | collision_threshold    | 0.01       | 碰撞检测安全距离阈值              | float    |
| **robot**                | kinematics             | 'acker'    | 运动学模型（acker /diff）         | str      |
|                          | wheelbase              | 0.67       | 阿克曼底盘轴距（m）               | float    |
|                          | max_speed              | [2.0, 2.0] | 最大前进 / 后退速度（m/s）        | list     |
|                          | max_acce               | [1.0, 1.0] | 最大加 / 减速度（m/s²）           | list     |
|                          | length                 | 0.95       | 车体长度（碰撞检测用）            | float    |
|                          | width                  | 0.67       | 车体宽度（碰撞检测用）            | float    |
| **ipath (initial path)** | interval               | 0.09       | 路径插值间隔（m）                 | float    |
|                          | waypoints              | []         | 输入全局路径点序列                | list     |
|                          | curve_style            | 'reeds'    | 路径生成方式（reeds/dubins/line） | str      |
|                          | min_radius             | 0.85       | 最小转弯半径（m）                 | float    |
|                          | loop                   | False      | 是否循环路径                      | bool     |
|                          | arrive_threshold       | 0.3        | 终点到达判定距离（m）             | float    |
|                          | close_threshold        | 0.1        | 靠近路径判定距离（m）             | float    |
|                          | arrive_index_threshold | 3          | 路径终点索引阈值                  | int      |
| **pan**                  | iter_num               | 2          | PAN 总迭代次数                    | int      |
|                          | dune_max_num           | 100        | DUNE 障碍距离场迭代次数           | int      |
|                          | nrmp_max_num           | 10         | NRMP 轨迹优化迭代次数             | int      |
|                          | dune_checkpoint        | None       | DUNE 预训练模型路径               | str      |
|                          | iter_threshold         | 0.1        | 迭代收敛停止阈值                  | float    |
| **adjust**               | q_s                    | 0.3        | 路径跟踪惩罚权重                  | float    |
|                          | p_u                    | 2.5        | 控制量平滑惩罚权重                | float    |
|                          | eta                    | 5.0        | 避障代价函数权重                  | float    |
|                          | d_max                  | 2.0        | 优化中的避障距离目标上限（m），不是点云检测范围 | float    |
|                          | d_min                  | 0.005      | 障碍最小安全距离（m）             | float    |

| 模块   | 参数名                 | 公式对应关系                                                 | 核心轨迹影响                                                 |
| ------ | ---------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| mpc    | receding               | 预测时域长度 H=10，轨迹长度 = H×step_time                    | 增大→轨迹更长、规划视野更远；减小→反应更快、更灵活           |
|        | step_time              | 控制步长 dt=0.2s，状态转移公式 st+1=Atst+Btut+ct             | 增大→轨迹更新频率降低；减小→更新更频繁、轨迹更精细           |
|        | ref_speed              | 基础代价 C0 中速度跟踪项参考值                               | 增大→轨迹巡航速度更快；减小→更平缓                           |
|        | device                 | 无直接公式关联，仅影响计算效率                               | 固定为 CPU，不影响轨迹形状                                   |
|        | time_print             | 无直接公式关联                                               | 仅控制日志输出，不影响轨迹                                   |
|        | collision_threshold    | 规划器估计最小净距低于该值时停车                             | 增大→更早停车；减小→允许更贴近障碍物                         |
| robot  | kinematics             | 运动学模型（acker = 阿克曼，diff = 差分），决定 At,Bt,ct 系数矩阵 | 切换模型→轨迹转弯逻辑改变（阿克曼有最小转弯半径，差分更灵活） |
|        | wheelbase              | 阿克曼模型中轴距参数，影响转向角与转弯半径关系               | 增大→最小转弯半径变大，轨迹转弯更平缓；减小→转弯更灵活       |
|        | max_speed              | 控制量约束 umin⪯uh⪯umax 中速度边界                           | 增大→轨迹最大速度上限提高；减小→更保守                       |
|        | max_acce               | 控制量变化率约束 amin⪯uh+1−uh⪯amax                           | 增大→轨迹加 / 减速更激进；减小→加速 / 减速更平缓             |
|        | length                 | 机器人凸集 Zt(st) 长度参数，影响距离计算 Di,t                | 增大→轨迹避障时预留空间更大；减小→更贴障碍物                 |
|        | width                  | 机器人凸集 Zt(st) 宽度参数，影响距离计算 Di,t                | 增大→轨迹避障时预留空间更大；减小→更贴障碍物                 |
| ipath  | interval               | 参考路径 sref 插值间隔                                       | 增大→参考路径点更稀疏，轨迹跟踪精度降低；减小→参考路径更密集、跟踪更精准 |
|        | waypoints              | 参考路径 sref 原始点序列                                     | 定义全局导航目标路径，直接决定轨迹的整体走向                 |
|        | curve_style            | 参考路径生成方式（reeds=Reeds-Shepp，dubins=Dubins，line = 直线） | 切换方式→参考路径曲率特性改变，进而影响跟踪轨迹的平滑度      |
|        | min_radius             | 参考路径最小转弯半径约束                                     | 增大→参考路径转弯更平缓，轨迹跟随更保守；减小→转弯更锐利     |
|        | loop                   | 参考路径是否循环                                             | True→轨迹循环行驶；False→到达终点后停止                      |
|        | arrive_threshold       | 终点到达判定：∥sT−sgoal∥≤0.3m                                | 增大→更早判定到达终点；减小→需更接近终点才判定完成           |
|        | close_threshold        | 靠近参考路径判定阈值                                         | 增大→对参考路径偏离容忍度更高；减小→更严格跟踪               |
|        | arrive_index_threshold | 终点索引判定阈值                                             | 影响终点判定的时机，不直接改变轨迹形状                       |
| pan    | iter_num               | DUNE 与 NRMP 交替迭代次数（感知 - 控制迭代）                 | 增大→轨迹优化更充分、精度更高；减小→计算更快、精度略降       |
|        | dune_max_num           | DUNE 网络内部障碍距离场迭代次数                              | 增大→LDF 特征更精准，避障轨迹更可靠；减小→计算更快、特征精度略降 |
|        | nrmp_max_num           | NRMP 轨迹优化迭代次数                                        | 增大→轨迹优化更充分、更平滑；减小→计算更快、优化程度降低     |
|        | dune_checkpoint        | DUNE 预训练模型路径，影响 LDF 特征生成                       | 加载优质预训练模型→避障特征更精准，轨迹避障效果更好          |
|        | iter_threshold         | 迭代收敛判定：∥Ck+1−Ck∥<0.1                                  | 增大→迭代更早停止；减小→收敛更充分、轨迹更稳定               |
| adjust | q_s                    | 基础代价 C0=∑qs∥s−sref∥2+pu∥u∥2 中轨迹跟踪权重               | 增大→轨迹更贴参考路径、不易偏离；减小→轨迹更灵活、易避障偏离 |
|        | p_u                    | 基础代价 C0 中控制量平滑权重                                 | 增大→轨迹控制动作更柔和、无急拐；减小→控制动作更激进、轨迹可能抖动 |
|        | eta                    | 避障代价 Cobs=η∑max(dmin−dist,0)2 中避障权重                 | 增大→避障优先级更高、轨迹绕障更明显；减小→避障激进、易贴障甚至碰撞 |
|        | d_max                  | 优化中的避障距离目标上限，不是障碍点过滤距离                | 增大→允许优化争取更大净距；实际效果还受权重、路径与约束影响 |
|        | d_min                  | 避障代价 Cobs 中最小安全距离                                 | 增大→轨迹离障碍物更远、更保守；减小→更贴障碍物、更激进       |

### 走直线左右摇摆问题

1. **减小max_acce的角加速度（小于0.1）**

   对 Ackermann 车辆，`max_acce` 的第二个值用于限制转向量的变化速度。数值过大时，优化器每次求解都可能快速改变转向角；当横向误差或航向误差存在噪声时，车辆容易出现“向左修正后马上向右修正”的左右摆动。

   适当减小该值可以让转向变化更平滑，减少控制输出的突变。

2. **增大 `receding` 或 `step_time`**

   `receding` 表示预测步数，`step_time` 表示每个预测步的时间，因此预测总时长约为：

   ```text
   prediction_time = receding × step_time
   ```

   参考速度降低后，如果 `receding` 不变，预测覆盖的实际距离也会变短，优化器看到的路径范围不足，容易只根据眼前的小误差反复修正，导致直线行驶左右摇摆。

3. **增大 `q_s`**

   `q_s` 是状态误差代价权重，用于提高车辆预测状态与参考路径之间的偏差惩罚。适当增大后，优化器会更重视横向位置和航向误差，减少车辆偏离路径后再大幅反向修正的情况。

## 5.4 NeuPAN Limo Gazebo 仿真配置（Docker/ROS1/本地虚拟机）


---

### 完整部署流程

#### 第一步：整理目录结构 (修复依赖位置)

确保 `limo_ros` 和 `rvo_ros` 位于 `src` 的根目录下。

```bash
cd /home/hart2204/RoutePlanning/neupan_ws

# 1. 移动误下载的包到 src 根目录 (根据实际情况调整源路径)
# 假设之前下在 example 文件夹下:
mv src/neupan_ros/example/gazebo_limo/limo_ros src/
mv src/neupan_ros/example/gazebo_limo/rvo_ros src/

# 2. 安装系统依赖
rosdep install --from-paths src --ignore-src -r -y
```

#### 第二步：清理环境与编译配置

清理旧缓存，并忽略不需要的真实硬件驱动。

```bash
cd /home/hart2204/RoutePlanning/neupan_ws

# 1. 彻底清理旧的编译缓存 (关键：解决 CMake 路径错误)
rm -rf build devel

# 2. 忽略真实雷达驱动 (关键：解决 ROS2 编译报错)
touch src/livox_ros_driver2/CATKIN_IGNORE

# 3. 开始编译
catkin_make
```

*看见 `[100%] Built target ...` 即表示编译成功。*

#### 第三步：配置环境变量 (关键)

为了让 Gazebo 动态障碍物生效，必须设置插件路径；为了解决网络报错，建议绑定 host。

```bash
# 建议写入 ~/.bashrc 以永久生效
echo "export GAZEBO_PLUGIN_PATH=\${GAZEBO_PLUGIN_PATH}:/home/hart2204/neupan_ws/devel/lib" >> ~/.bashrc

# [解决 RLException 网络报错] 强制指定主机名
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc

# 刷新配置
source ~/.bashrc
```

### 启动仿真

开启两个终端分别运行环境与算法。

#### 终端 1：启动 Gazebo 仿真环境

加载 Limo 小车模型和包含动态障碍物的世界地图

```bash
cd /home/hart2204/RoutePlanning/neupan_ws
source devel/setup.bash

# 启动环境 Launch 文件
roslaunch neupan_ros gazebo_limo_env_complex_20.launch
```

#### 终端 2：启动 Neupan 算法节点

启动核心规划算法、Rviz 可视化界面。

```bash
cd /home/hart2204/RoutePlanning/neupan_ws
source devel/setup.bash

# 启动算法 Launch 文件
roslaunch neupan_ros neupan_gazebo_limo.launch
```

### 验证与故障排除

#### 验证步骤

1. **观察 Rviz**：启动成功后，Rviz 界面应显示白色雷达点云、彩色圆柱体障碍物和小车模型。
2. **发送指令**：点击 Rviz 顶部工具栏的 **"2D Nav Goal"**，在地图空旷处点击并拖动方向。
3. **结果**：小车应自动规划出路径（红/绿线）并避开障碍物行驶至目标点。

#### 常见报错速查表

| 报错信息                                                     | 原因                          | 解决方案                                              |
| ------------------------------------------------------------ | ----------------------------- | ----------------------------------------------------- |
| `RLException: Unable to contact my own server`               | 主机名无法解析                | 执行 `export ROS_HOSTNAME=localhost`                  |
| `Resource not found: limo_description`                       | 未 source 或包未在 src 根目录 | 检查目录结构并 source                                 |
| `Could not find a package configuration file provided by "ament_cmake_auto"` | 误编译了 ROS2 包              | `touch src/livox_ros_driver2/CATKIN_IGNORE`           |
| Gazebo 里的障碍物静止不动                                    | 找不到 RVO 插件               | `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:...` |
| **刚启动的时候会有识别没有障碍物产生空张量**                 |                               | **可以添加一个极其远的虚假点来规避这个问题**          |

##### 空张量解决办法：

现在，当 [obstacle_points](vscode-file://vscode-app/d:/Microsoft VS Code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 为 `None` 时（即没有检测到障碍物时），neupan_core.py代码会使用一个位于 (10000.0, 10000.0) 的虚假障碍物点，而不是一个空数组。这将避免空张量的问题。

```python
            if self.obstacle_points is None:
                rospy.logwarn_throttle(
                    1, "No obstacle points, only path tracking task will be performed"
                )
                obs = np.array([[10000.0], [10000.0]])
            else:
                obs = self.obstacle_points

            action, info = self.neupan_planner(self.robot_state, obs)
```


# 六、TEB 与其他全局算法


## 6.1 TEB 的 ROS1 Noetic 安装

Ubuntu 20.04 + ROS Noetic：

```bash
sudo apt install ros-noetic-teb-local-planner
```

或：[rst-tu-dortmund/teb_local_planner: An optimal trajectory planner considering distinctive topologies for mobile robots based on Timed-Elastic-Bands (ROS Package)](https://github.com/rst-tu-dortmund/teb_local_planner?utm_source=chatgpt.com)

## 6.2 其他全局规划算法

### A*

#### 基于栅格地图

使用GlobalPlanner的A*

```yaml
base_global_planner: global_planner/GlobalPlanner
base_local_planner: teb_local_planner/TebLocalPlannerROS

controller_frequency: 10.0
controller_patience: 3.0
planner_frequency: 1.0
planner_patience: 5.0
oscillation_timeout: 10.0
oscillation_distance: 0.3
shutdown_costmaps: false
clearing_rotation_allowed: false

GlobalPlanner:
  allow_unknown: true
  default_tolerance: 1.0
  use_dijkstra: false
  use_quadratic: true
  use_grid_path: false
  old_navfn_behavior: false

```

#### 基于 OSM

使用自己写的a*算法（osm_map_handler/global_astar_node）


# 七、URDF、Gazebo 与 Docker 迁移


[solidworks导出urdf文件/导入ros详细教程_solidworks转urdf-CSDN博客](https://blog.csdn.net/hhhon/article/details/148581041)

[从零开始仿真ROS小车（一）urdf模型+rviz可视化_ros搭建小车rviz摄像头status黄标-CSDN博客](https://blog.csdn.net/weixin_52029211/article/details/119538435)

[solidworks转URDF，并且在rviz中仿真_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1LV4y17734/?vd_source=1aac97640b2d7f2e6ae51b4b1cac54e5)

[SolidWorks模型导出urdf （古月居老师）_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1Tx411o7rH/?spm_id_from=333.337.search-card.all.click&vd_source=1aac97640b2d7f2e6ae51b4b1cac54e5)

[阿克曼结构移动机器人的gazebo仿真（一） - 哔哩哔哩](https://www.bilibili.com/opus/625943392648214986)

也可以直接基于现成的模板来修改！！！

## 7.1 Gazebo 仿真

### 虚拟机

[Ubuntu20.04+ROS1+PX4+Gazebo仿真（三）环境配置 - 哔哩哔哩](https://www.bilibili.com/opus/934654041226477571)

[【实战教程】Ubuntu20.04 下 PX4+Gazebo+Mid360+Tare Planner 全流程搭建_px4连gazebo-CSDN博客](https://blog.csdn.net/AresMHT/article/details/157025477)

```bash
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git

#进入PX4-Autopilot根目录
cd PX4-Autopilot

#更新子模块
git submodule update --init --recursive

cd ..

#安装工具链，依赖包
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

### 更新 ROS GPG 密钥

```bash
# 1. 删除旧的 ROS 源列表（如果存在）
sudo rm /etc/apt/sources.list.d/ros*.list

# 2. 重新添加 ROS 软件源并更新密钥
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. 添加 ROS 源到 sources.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros1.list

# 4. 更新软件包列表
sudo apt update

# 5. 重新运行 PX4 安装脚本
bash ./Tools/setup/ubuntu.sh
```

### 使用教程

```bash
#模型下载
git clone https://gitee.com/bingda-robot/gazebo_models.git

#模型导入，gazebo里面insert
sudo cp -r /home/hart2204/gazebo_models/* /root/.gazebo/models/

#world文件为机器人的仿真环境

```

## 7.2 打包 Docker 迁移到 NUC




# 八、地图、SLAM 与全局规划

## 8.1 OSM 地图

[www.openstreetmap.org](https://www.openstreetmap.org/)

![image-20260302173744876](../mdPicture/image-20260302173744876.png)

## 8.2 SLAM 建图与地图转换

### 地图说明

全局一定要有地图，可以是瓦片可以是slam地图

### SLAM 建图

#### FAST_LIO2 纯雷达(nuc)

```bash
Neupan
conda activate lxw
cd /home/nuc/lxw
mkdir -p fast_lio_ws/src
cd fast_lio_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git

# 1. 进入 FAST_LIO 源码目录
cd /home/nuc/lxw/fast_lio_ws/src/FAST_LIO
# 2. 初始化并更新子模块（拉取 ikd-Tree 的代码）
git submodule update --init --recursive
```

##### 修改

[Livox激光雷达 mid360 跑 fastlio2 - 流程记录 - 技术栈](https://jishuzhan.net/article/1769017370111643649#2.2、修改代码)

[文档预览 - Gitee.com](https://gitee.com/gwmunan/ros2/wikis/pages?sort_id=12161327&doc_id=4855084)

主要是把 `livox_ros_driver` 改成 `livox_ros_driver2`

1、`fast_lio2_ws/src/FAST_LIO` 下的 `CMakeLists.txt` 与`package.xml`

2、laserMapping.cpp

3、preprocess.cpp、preprocess.h

##### Build

```bash
# 先 Source 包含有 livox 驱动的原始工作区
source /home/nuc/lxw/neupan_ws/devel/setup.bash
conda activate lxw
catkin_make -DPYTHON_EXECUTABLE=/root/miniconda3/envs/lxw/bin/python
```

##### 启动

```bash
#启动 Mid360 驱动
conda activate lxw
cd /home/nuc/lxw/fast_lio_ws
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
roslaunch livox_ros_driver2 msg_MID360.launch

#启动 FAST_LIO 建图
conda activate lxw
cd /home/nuc/lxw/fast_lio_ws
source /home/nuc/lxw/neupan_ws/devel/setup.bash
source /home/nuc/lxw/fast_lio_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

##### pcd地图

```bash
#查看
cd /home/nuc/lxw/fast_lio_ws/src/FAST_LIO/PCD/
pcl_viewer scans.pcd
```

将 3D 点云（PCD）转换为 2D 栅格地图（PGM），主要思路是沿着 Z 轴（高度）切取一个切片（高度范围内的点汇聚投影到 2D 平面上，有障碍物的地方标黑，没有的地方标白），然后输出可以用作 ROS 导航（Navigation 栈）的 `.pgm` 图像和配套的 `.yaml` 描述文件。

```bash
git clone https://github.com/Hinson-A/pcd2pgm_package.git
```

修改launch

```html
<launch>
    <node pkg="pcd2pgm" name="pcd2pgm" type="pcd2pgm" output="screen">
        <!-- 要转换的原始pcd文件存放位置，修改为你保存地图的实际位置 -->
        <param name="file_directory" value= "/home/nuc/lxw/fast_lio_ws/src/FAST_LIO/PCD/" />
        <!-- 原始pcd文件的名字，比如 scans.pcd -->
        <param name="file_name" value= "scans" />

        <!-- 关键参数配置 -->
        <param name="thre_z_min" value= "0.2" />
        <param name="thre_z_max" value= "1.5" />

        <!-- 直通滤波标志位：0表示根据上述z轴阈值过滤，生成2D地图 -->
        <param name="flag_pass_through" value= "0" />

        <!-- 栅格地图分辨率 -->
        <param name="map_resolution" value= "0.05" />

        <!-- 发布的话题名称 -->
        <param name="map_topic_name" value= "map" />
    </node>
</launch>
```

```bash
cd /home/nuc/lxw/fast_lio_ws
conda activate lxw
catkin_make -DPYTHON_EXECUTABLE=/root/miniconda3/envs/lxw/bin/python
source devel/setup.bash

#在一个终端中启动 pcd2pgm 将 PCD 转换为 ROS 里占用的 Map Topic 发送出来：
roslaunch pcd2pgm run.launch

#在另一个终端中使用 map_server 将发出话题保存为磁盘文件：
#创建一个专门保存最终 2D 栅格地图的文件夹（比如 ~/lxw/maps），然后进入该文件夹执行地图保存节点：
mkdir -p /home/nuc/lxw/maps && cd /home/nuc/lxw/maps
rosrun map_server map_saver -f my_indoor_map
```

****


#### FAST_LIVO2 雷达+视觉(nuc)

```bash
Neupan
conda activate lxw
cd /home/nuc/lxw
mkdir -p fast_livo_ws/src
cd fast_livo_ws/src
git clone https://github.com/hku-mars/FAST-LIVO2
```

##### 先决条件

###### 1、Sophus

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

报错：c++版本问题，修改源代码（编辑 `/home/nuc/lxw/Sophus/sophus/so2.cpp` 第 32-33 行）

```c++
// 原代码（错误）
unit_complex_.real() = 1.;
unit_complex_.imag() = 0.;

// 修改为
unit_complex_ = std::complex<double>(1., 0.);
```

###### 2、Vikit

```bash
# Different from the one used in fast-livo1
cd catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git
```

##### mid360文件创建

**修改它的驱动依赖为 `livox_ros_driver2`**

![image-20260310170708340](../mdPicture/image-20260310170708340.png)

请找到 CMakeLists.txt 大约第 79 行的 `find_package` 部分，将它加上一行 `livox_ros_driver2`：

```cmake
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2 #<--添加
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
)
```

#### LIO_SAM

[LIO-SAM 配置 MID360 激光雷达完全指南_lio-sam-mid360-CSDN博客](https://blog.csdn.net/qq_33346065/article/details/149789646)

```bash
Neupan
conda activate lxw
cd /home/nuc/lxw
mkdir -p lio_sam_ws/src
cd lio_sam_ws/src
git clone -b Livox-ros-driver2 https://github.com/nkymzsy/LIO-SAM-MID360.git
cd LIO-SAM-MID360
git submodule update --init
cd /home/nuc/lxw/lio_sam_ws
source /home/nuc/lxw/neupan_ws/devel/setup.bash
catkin_make
source /home/nuc/lxw/lio_sam_ws/devel/setup.bash
roslaunch lio_sam run6axis.launch  # 使用内置六轴IMU
```

##### 依赖与bug解决

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

```bash
sed -i 's|#include <opencv/cv.h>|#include <opencv2/opencv.hpp>|g' /home/nuc/lxw/lio_sam_ws/src/LIO-SAM-MID360/include/utility.h

sed -i 's|set(CMAKE_CXX_FLAGS "-std=c++11")|set(CMAKE_CXX_STANDARD 14)|g' \
    /home/nuc/lxw/lio_sam_ws/src/LIO-SAM-MID360/CMakeLists.txt

# 在 OpenCV 头文件前添加 PCL kdtree_flann.h
sed -i 's|#include <opencv2/opencv.hpp>|#include <pcl/kdtree/kdtree_flann.h>\n#include <opencv2/opencv.hpp>|g' \
    /home/nuc/lxw/lio_sam_ws/src/LIO-SAM-MID360/include/utility.h
```

##### 配置文件

```yaml
# Sensor Settings
sensor: livox                              # 设置为livox
N_SCAN: 4                                  # MID360的线束数量
Horizon_SCAN: 6000                         # 水平分辨率
downsampleRate: 1                          # 下采样率
lidarMinRange: 1.0                         # 最小雷达范围
lidarMaxRange: 100.0                       # 最大雷达范围

# IMU参数 (如果使用内置IMU)
imuTopic: "/livox/imu"                     # IMU话题
imuAccNoise: 0.01                          # 加速度计噪声
imuGyrNoise: 0.001                         # 陀螺仪噪声
imuAccBiasN: 0.0002                        # 加速度计偏置噪声
imuGyrBiasN: 0.00001                       # 陀螺仪偏置噪声
```

##### 地图保存位置

paramsLivoxIMU.yaml

```yaml
savePCDDirectory: "/Downloads/LOAM/"

savePCDDirectory: "/home/nuc/lxw/lio_sam_ws/src/LIO-SAM-MID360/PCD"
```


#### Mid360离线建图与生成栅格地图流程

记录了如何使用真实的 Livox Mid360 雷达收集数据，利用 FAST_LIO 算法进行离线建图，并最终将 3D 点云转换为适用于 ROS Navigation (导航栈) 的 2D 栅格地图（`.pgm` + `.yaml`）。

---

##### 阶段一：在真实环境中录制数据包（Rosbag）
此阶段需要带着雷达前往真实环境（如走廊、房间）。

1. **原点静止初始化**
   在启动雷达和电脑前，将雷达放在起点**静止放置5~10秒钟**，确保底层IMU能够完美初始化。

2. **启动雷达驱动**
   _打开终端1：_

   ```bash
   conda activate lxw
   cd /home/nuc/lxw/neupan_ws
   sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
   source devel/setup.bash
   roslaunch livox_ros_driver2 msg_MID360.launch
   ```

3. **开始录制数据**
   _打开终端2：_

   ```bash
   conda activate lxw
   cd /home/nuc/lxw/bags
   # 录制雷达和IMU数据到一个名叫 corridor_data.bag 的文件中
   rosbag record -O corridor_data.bag /livox/lidar /livox/imu
   ```

4. **开始移动**
   - 保持平稳、匀速（0.5~1 m/s）。
   - 转弯时尽量缓慢。
   - 走廊这种退化场景容易跑飞，最好走到尽头后原路返回起点形成闭环操作。
   - 如果是长白墙走廊，可以在地上随机放置纸箱作为特征点（后续在 2D 图像中擦除即可）。

5. **结束录制**
   回到起点后，首先静止停放2秒，切回**终端2**，按下 `Ctrl+C` 停止录制。

---

##### 阶段二：回放数据并使用 FAST_LIO 生成 3D 点云地图
此阶段可离线在工位上操作。前提条件：确保 `fast_lio_ws/src/FAST_LIO/config/mid360.yaml` 里面的 `pcd_save_en: true` 功能已经打开。

1. **启动建图节点**
   _打开终端1：_

   fast_lio:

   ```bash
   conda activate lxw
   cd /home/nuc/lxw/neupan_ws
   source devel/setup.bash
   roslaunch fast_lio mapping_mid360.launch
   ```
   （此时只启动了建图程序和 RViz，界面还没有数据传入）

   或lio_sam：

   ```bash
   conda activate lxw
   cd /home/nuc/lxw/lio_sam_ws
   source /home/nuc/lxw/lio_sam_ws/devel/setup.bash
   roslaunch lio_sam run6axis.launch  # 使用内置六轴IMU
   ```

2. **播放录制好的数据**
   _打开终端2：_

   ```bash
   conda activate lxw
   cd /home/nuc/lxw/bags
   # --clock 确保程序使用bag的时间，-d 3 代表延迟3秒开始播放
   rosbag play corridor_data.bag --clock -d 3
   ```

3. **保存PCD地图文件**
   当终端2提示播放完毕后，**回到终端1**，按下 `Ctrl+C` 用以停止建图程序。
   此时会自动触发地图保存功能。生成的点云文件会被默认保存至该路径下：
   `/home/nuc/lxw/fast_lio_ws/src/FAST_LIO/PCD/scans.pcd`

---

##### 阶段三：降维切片生成 2D 导航地图 (PGM)
将 3D 点云图通过指定 Z 轴高度区间过滤，沿垂直向下投影到 2D 栅格中。

1. **运行转换层程序**
   确保 `run.launch` 文件中 `thre_z_min` 和 `thre_z_max` 截取的高度避开了地面和天花板。
   _打开终端1：_
   ```bash
   conda activate lxw
   cd /home/nuc/lxw/neupan_ws
   source devel/setup.bash
   roslaunch pcd2pgm run.launch
   ```

   ```xml
   <!--param name="file_directory" value= "/home/nuc/lxw/fast_lio_ws/src/FAST_LIO/PCD/" /-->
   <param name="file_directory" value= "/home/nuc/lxw/lio_sam_ws/src/LIO-SAM-MID360/PCD/" /-->
   <!-- 原始pcd文件的名字，比如 scans.pcd;GlobalMap.pcd >
   <param name="file_name" value= "GlobalMap" />
   ```

2. **保存为地图文件**
   _打开终端2：_

   ```bash
   cd /home/nuc/lxw/maps
   
   # 保存地图节点，地图的本体前缀命名为 my_corridor
   rosrun map_server map_saver -f my_corridor
   ```

执行完毕后，` /home/nuc/lxw/maps` 文件夹下便会生成 `my_corridor.pgm` 和 `my_corridor.yaml`，此文件可直接被加载进导航配置中。

## 8.3 NeuPAN 与全局规划的关系

neupan做的是局部的规划和避障

如果要全局的话常规的要设计
全局地图 ：
栅格地图或点云地图。
告诉机器人综合楼在哪里，路网是什么样的。
全局规划器 ：
比如 A*、Dijkstra 或 Hybrid A*。
它的作用是在全局地图上搜索出一条从起点到综合楼的粗略路径。
这些路径点会作为参考路径喂给 NeuPAN。

![image-20260121122150811](../mdPicture/image-20260121122150811.png)

## 8.4 Hybrid A*

1、[teddyluo/hybrid-a-star-annotation: Hybrid A*路径规划器的代码注释](https://github.com/teddyluo/hybrid-a-star-annotation?tab=readme-ov-file)

2、[dengpw/hybrid_astar_planner: This is a global planner plugin of ROS move_base package](https://github.com/dengpw/hybrid_astar_planner)

### neupan结合1的混合A*

```bash
#1. 系统级依赖安装 (OMPL)
sudo apt update
sudo apt install libompl-dev

# 克隆代码到你的工作空间
cd /home/nuc/lxw/neupan_ws/src
git clone https://github.com/teddyluo/hybrid-a-star-annotation.git
```

```cmake
#修改1中功能包的cmakelist文件的ompl查找如下
# 查找 OMPL（支持大小写不同的配置文件名）
find_package(ompl QUIET)
if(NOT ompl_FOUND)
    find_package(OMPL QUIET)
endif()

if(NOT ompl_FOUND AND NOT OMPL_FOUND)
    message(FATAL_ERROR "OMPL not found!")
endif()

# 统一变量名（兼容不同版本）
if(ompl_FOUND)
    set(OMPL_LIBRARIES ${ompl_LIBRARIES})
    set(OMPL_INCLUDE_DIRS ${ompl_INCLUDE_DIRS})
    set(OMPL_FOUND TRUE)
endif()
```

```bash
#测试该功能包
rospack find hybrid_astar
rosrun hybrid_astar hybrid_astar
```

### 问题

#### 一、路径输入层优化建议

##### 问题原因

Hybrid A* 输出路径点过于密集，导致 NeuPAN 生成的 Dubins/Reeds 路径频繁切向、曲率突变，最终引发轨迹震荡和偏移。

##### 改进方法

1. 对 Hybrid A* 全局路径执行**稀疏化降采样**，仅保留关键节点：
   - **必留节点**：路径拐点、方向变化点、起点、终点
   - **直线段处理**：仅保留起点与终点，删除中间冗余点
   - **最终约束**：输入 NeuPAN 的 waypoints 数量控制在 **5～15 个**
2. 降采样逻辑：采用「距离阈值法」（相邻点间距≥0.5m 保留）或「角度阈值法」（方向变化≥15° 保留）

---

#### 二、优化权重参数调整建议

##### 问题原因

NeuPAN 默认路径跟踪权重为弱约束，无障碍环境下优化器优先满足「终点到达」而非「路径贴合」，导致漂移。

##### 改进方法

| 参数类别       | 默认值范围      | 推荐调整值      | 调整逻辑                     |
| -------------- | --------------- | --------------- | ---------------------------- |
| 位置跟踪权重 q | [0.5, 0.5, 0.5] | [8.0, 8.0, 4.0] | 放大 8~10 倍，强化路径贴合   |
| 速度跟踪权重 p | [0.3, 0.3]      | [2.0, 2.0]      | 放大 3~5 倍，稳定行驶速度    |
| 转向平滑权重   | [0.1, 0.1]      | [1.5, 3.0]      | 放大 10~15 倍，避免转向抖动  |
| 航向跟踪权重   | [0.5, 0.5]      | [0.3, 0.5]      | 适当降低，适配阿克曼转向特性 |

# 九、点对点导航

## 9.1 栅格地图 + NeuPAN + Hybrid A*：室内走廊导航

```bash
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
# 启动整套（室内）
roslaunch neupan_ros full_system_run.launch
# 在 RViz 用 2D Nav Goal 点目标，观察 /sPath 和 /neupan_plan，确认 /neupan_cmd_vel 发出
```

验证

```bash
#检查环境变量
echo $ROS_PACKAGE_PATH | tr ':' '\n'

# 地图被 map_server 发布
rostopic list | grep /map
rostopic echo -n1 /map

# Hybrid A* 是否运行并发布路径（检查 /path /sPath）
rostopic list | grep -E "/path|/sPath"
rostopic echo -n1 /sPath || rostopic echo -n1 /path

# NeuPAN 是否收到初始路径并发布局部计划
rostopic list | grep /neupan_plan
rostopic echo -n1 /neupan_plan

# TF 检查 (map -> base_link)
rosrun tf tf_echo /map /base_link

# 查看运行节点与连接
rosnode list
rosrun rqt_graph rqt_graph   # 可视化节点/话题连接（需图形界面）
```

## 9.2 OSM + GPS + NeuPAN/A*：室外点到点导航

`TEB` 测试指令是：

```bash
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
roslaunch teb_test outdoor_teb_test.launch
```

“不带任何局部规划”的全局直跟测试，也就是你之前一直在跑的 direct tracker，是：

```bash
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
roslaunch neupan_ros outdoor_global_only_test.launch direct_tracker_yaw_offset_deg:=0 direct_tracker_projected_goal_stop_tolerance:=1.0
```

如果你想测带局部避障的 `NeuPAN` 全系统，则是：

```bash
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0
roslaunch neupan_ros full_system_run.launch outdoor_mode:=true
```

简单对应关系是：

- `teb_test/outdoor_teb_test.launch`：`OSM A* + TEB`
- `neupan_ros/outdoor_global_only_test.launch`：`OSM A* + 不带局部避障，仅直接跟踪`
- `neupan_ros/full_system_run.launch`：`OSM A* + NeuPAN 局部规划`

## 9.3 高德车道级地图 + GPS + NeuPAN：室外点到点导航

[路径规划-基础 API 文档-开发指南-Web服务 API | 高德地图API](https://lbs.amap.com/api/webservice/guide/api/direction)

![image-20260420201842319](../mdPicture/image-20260420201842319.png)

keyname：广工lxwcar-室外导航

key：<AMAP_KEY>

私钥：<AMAP_SECRET>

```text
高德地图
   ↓
A*全局路径
   ↓
局部裁剪（30m）
   ↓
坐标转换（→ LiDAR系）
   ↓
Occupancy Grid
   ↓
路径优化（贴地）
   ↓
走廊生成
   ↓
NeuPAN
   ↓
MPC控制
```

```text
/osm_map_node          （加载高德/OSM路网）
        ↓
/global_planner_node   （A*）
        ↓
/path_smoother_node    （样条平滑）
        ↓
/path_cropper_node     （局部裁剪 30~50m）
        ↓
-------------------融合层-------------------
        ↓
/lio_sam / fast_lio    （LiDAR SLAM）
        ↓
/map_fusion_node       （路径贴合/走廊生成）
        ↓
-------------------局部规划-------------------
        ↓
/neupan_node           （局部轨迹）
        ↓
/vehicle_interface_node（底盘）
```

## 9.4 开源地图 API：天地图

[天地图API](http://lbs.tianditu.gov.cn/server/MapService.html)

[swri-robotics/mapviz: Modular ROS visualization tool for 2D data.](https://github.com/swri-robotics/mapviz)

```text
http://t0.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.
0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={level}&TIL
EROW={x}&TILECOL={y}&tk=56127e90e994e759a9dbab1a1f3bfd5a
```

### 下载maprviz

先看你的 ROS 版本：

```bash
echo $ROS_DISTRO
```

然后安装：

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTR
```

### 开启g90 maprviz

```bash
roslaunch wheeltec_gps_driver mapviz_path.launch
```

# 十、跟踪算法与 Nav2

![image-20260720200927233](../mdPicture/image-20260720200927233.png)

## 10.1 Nav2

[Route Server — Nav2 1.0.0 documentation](https://docs.nav2.org/configuration/packages/configuring-route-server.html?utm_source=chatgpt.com)

[ros-navigation/navigation2: ROS 2 Navigation Framework and System](https://github.com/ros-navigation/navigation2)

在室外路网地图导航中，可将 OSM 或高精地图构建为拓扑路网（Route Graph），并结合 GPS/RTK 提供的全局定位信息，将车辆实时位置转换到地图坐标系中作为导航起点；在此基础上，引入 Nav2 Route Server 在路网图上进行全局路径规划，通过图搜索算法（如 A* 或 Dijkstra）生成符合道路约束的导航路径。该路径作为上层路由结果输入到 Nav2 的局部规划与控制模块中，由控制器（如 DWB、TEB 或自定义 MPC）结合实时感知信息生成可执行轨迹并进行动态避障，从而形成“GPS定位 + 路网级全局规划 + Nav2局部规划与控制”的分层导航体系，实现室外环境中沿合法道路行驶的自主导航。

# 十一、RTK/CORS

## 11.1 中国移动 CORS

卡号：<CORS_USER>
密码：<CORS_PASSWORD>

【cors账号配置方法】
CORS设置方式：
①IP：120.253.239.161
②端口：8001对应CGCS2000坐标系，8002端口对应WGS84坐标系，记得中央子午线改成当地
③源节点：
RTCM33_GRCEJ（五星十六频，推荐五星仪器以上使用）
RTCM33_GRCEpro（四星十三频，推荐四星仪器以上使用）
RTCM33_GRCE（四星十一频，推荐四星仪器以上使用）
RTCM33_GRC（三星八频，推荐三星仪器使用）
RTCM30_GR（双星仪器使用）

### **STRSVR 工具配置（测试使用）**

![image-20260511164238416](../mdPicture/image-20260511164238416.png)

![image-20260511164434445](../mdPicture/image-20260511164434445.png)

![image-20260511164530846](../mdPicture/image-20260511164530846.png)

### 4G DTU

```text
AT+NTRIP=addr,port,user,password,mount
```

(其中 addr 为 Ntrip 服务器地址，port 为端口号，user 与 password 是 CORS 账号和

密码，mount 为挂载点。)

例如：

```text
AT+NTRIP=120.253.239.161,8002,<CORS_USER>,<CORS_PASSWORD>,RTCM33_GRCE

```

**按一下”shift+Enter”键后再点击发送，**发送成功后会返回【OK】然后保存参数

挂载点从上面的实际源节点中选择，例如 `RTCM33_GRCE`。

## 11.2 G90 位置与双天线定向状态（2026-10-14 更新）

“GPS 定位状态”和“经纬度解状态”描述的是位置解；另外需要独立判断的是
双天线定向解。位置固定不能代替定向固定，位置浮动也不能直接推断定向无解。
桥底能否保持定向固定需实测，不能仅凭 `/heading` 持续更新判断。

### 驱动与接口

当前仍使用 `wheeltec_gps_driver` 的 NMEA 串口驱动，额外解析设备已经输出的
`#PVTSLNA`，无需切换到另一套 Unicore 驱动。新增解析先检查 Unicore CRC32、
报文长度、数值范围及有限性，再分别提取位置类型和定向类型。

| 接口 | 类型 | 内容 |
|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | 经纬度、协方差及 ROS 定位状态；当前驱动把 GGA 固定解和浮动解都映射成 `status.status=2`，不能只凭该字段判断固定 |
| `/gnss/gpgga` | `nmea_msgs/Gpgga` | `gps_qual` 位置质量、卫星数和 HDOP |
| `/heading` | `geometry_msgs/QuaternionStamped` | 经原安装偏角和ENU转换后的航向；默认只从定向Fixed且有效的同一条PVTSLNA报文发布，不再用HDT借旧状态 |
| `/gnss/solution_status` | `std_msgs/String` | JSON，包含独立位置/定向解类型、有效性及整数 `stamp_ns`，与同报文航向Header完全对应 |

相关实现：

- `src/wheeltec_gps_driver/src/libnmea_navsat_driver/unicore.py`：PVTSLNA 字段解析与 CRC32。
- `src/wheeltec_gps_driver/src/libnmea_navsat_driver/driver.py`：从同一PVTSLNA发布航向及解状态；Fixed模式不再将HDT与前一条解状态关联。
- `src/robot_base_driver/scripts/global_odom_bridge.py`：定位保护独立检查定向状态。
- `src/robot_base_driver/scripts/system_run_logger.py`：记录解状态及更新年龄。

**2026-10-21更正与诊断：** 用户已确认同一设备、同一串口配置10 Hz且已生效；旧bag/JSONL仅证明当时ROS解状态约1 Hz，不能据此要求再次提频。保持设备配置和原0.5 s航向保护不变，先核对实际串口出包、读取/解码、解析和ROS发布。

新GPS驱动沿原来的唯一串口读取路径统计，每秒在终端汇总，并发布 `/gnss/serial_diagnostics`。重启GPS驱动及综合日志节点后检查：

```bash
rostopic echo /gnss/serial_diagnostics
rostopic hz -w 200 /gnss/solution_status
rostopic hz -w 200 /heading
```

`/heading`仅在有效定向时发布。诊断的 `window` 是本周期计数，`total` 是本次驱动会话累计，`rates_per_sec` 按实际单调时钟间隔计算，不是假定每条报文1 Hz；诊断话题自身1 Hz不代表GPS出包1 Hz。`session_id`/`sequence`用于分辨驱动重启或诊断漏收。即使串口正在等待2 s读取超时，也有独立的1 s统计心跳。

- `read_bytes/complete_lines/partial_reads/ascii_decode_errors`：去空白和解码前的实际读入字节、以换行结束的读取、未以换行结束的读取及解码失败。
- `family_*`、`raw_pvtslna_prefix/embedded_pvtslna/alternate_pvtsln`：各类报文，以及标准前缀、行中嵌入前缀、其他PVTSLN格式；辅助报文也纳入流量，不擅自忽略其负载。
- `pvtslna_recognized/pvtslna_parse_ok/pvtslna_parse_errors/pvtslna_crc_errors`：识别、解析成功、解析失败及其中的CRC失败。
- `solution_publish_calls/heading_publish_calls` 和相应 `*_publish_errors`：Publisher调用成功/失败；包含错误解状态，不代表ROS订阅者已收到，也不等于Fixed帧数。
- `pvtslna_receive_age_sec`、`pvtslna_max_receive_gap_sec`：主机识别时刻的年龄及已观测最大间隔，不是GNSS测量时间；无新包时还要看年龄，不能只看平均Hz。
- `metadata`：实际串口、波特率、Python路径、PID、读取/解析模块绝对路径和SHA256；115200/8N1的名义数据预算11520 B/s仅供比对，主机已读流量不能证明设备总发送量或丢包率。

完整汇总自动写入原 `tracker_logs/system_run_full_*.jsonl` 的 `gnss_serial_diagnostics_event`，10 Hz采样仅留简要摘要，不另建一套日志。三份现有录包配置已加入该话题，非必需话题，不影响旧包验收要求。终端汇总可通过GPS launch参数 `serial_diagnostics_print:=false`关闭，话题和记录不关闭。

整车室外启动默认measurement＋auto：NUC内核同步，PVTSLNA FINE与GGA有效UTC、新历元的源龄和读取时间连续一致至少2 s后自动放行时间门控，不需要到室外改参数。UNKNOWN/无UTC只在启动时等待，仍需满足原定向Fixed和定位初始化条件；启动校时可重新等待，运行后钟跳/失去同步则锁存保护需停车重启。事件仍逐条进入原JSONL/rosbag；自动一致性检查不是硬同步或精度证明，不自动修改系统时钟，也不使用旧 `gps_use_gnss_time`。见 [时间整改与验收](PROJECT_ISSUES_CN.md#22-测量时间对齐软件链路已实现实车切换待验收)。

本次只加计数，不保存原始串口全文、不新增第二个读取进程、不发送设备配置命令。停车采集完整原始字节仍是后续步骤；只凭统计不能重建旧数据或证明接收机配置错误。详见 `PROJECT_ISSUES_CN.md` 第3.5节。

接收端要求GPS投影/fix/GGA源时间完全一致；完整航向校正还要求航向/解状态同整数 `stamp_ns`，不再容忍相邻50 ms位置帧借质量。该帧位置质量或新航向自身质量未齐时有界等待，不提前消费该帧。位置和定向状态分别判断：定向非Fixed时驱动不发布有效航向；初始化后，合格Fixed/Float位置仍可独立进入位置-only窗口，满足原稳定性与纠偏条件后续期，yaw不变。未初始化时仍需原Fixed定向条件。

最新源健康状态独立记录在 `/gps_guard/status.latest_heading_health`。即使旧Fixed航向尚未超时，新的定向Float、无效或CRC错误也会使完整Fixed跟踪降级；历史选择不能遮盖新失效、不能用后来的Fixed回填旧Float。已知失效后的乱序/重复旧Fixed不得恢复完整跟踪。原GPS诊断、0.5 s时效保留；全系统续推预算现为200 m/600 s（10月22日用户调整，独立底层入口仍默认100 m）。旧bag缺少同帧标识时，不放行完整航向校正。

协议字段依据：N4 Products Commands and Logs Reference Book V2 R1.2，
7.3.23 节表 7-81。`bestpos_type` 是位置类型，`heading_type` 是定向类型。
G90 用户手册第 44 页的 `gps_qual` 列表只说明位置状态，第 45 页介绍
`PVTSLNA` 定位定向报文；详细字段要查数据接口协议。

### 具体状态与三个布尔字段

位置类型 `position_type` 常见值如下；设备不一定会输出表中每一种类型。

| 值 | 含义 |
|---|---|
| `NONE` | 无位置解 |
| `SINGLE` | 单点定位 |
| `PSRDIFF` / `SBAS` | 伪距差分 / 星基增强定位 |
| `L1_FLOAT` / `IONOFREE_FLOAT` / `NARROW_FLOAT` | 不同类型的浮动解 |
| `L1_INT` / `WIDE_INT` / `NARROW_INT` | L1 / 宽巷 / 窄巷固定解 |

双天线定向类型 `heading_type` 主要关注：

| 值 | 含义 | 当前默认处理 |
|---|---|---|
| `NONE` | 无定向解 | 拒绝航向 |
| `NARROW_FLOAT` | 定向浮动解 | 拒绝航向 |
| `NARROW_INT` | 定向固定解 | 继续检查有效性和更新时间 |

布尔字段都只有 `true` / `false`：

| 字段 | 当前代码的 `true` 条件 |
|---|---|
| `position_fixed` | `position_type == "NARROW_INT"` |
| `heading_fixed` | `heading_type == "NARROW_INT"` |
| `heading_valid` | `heading_fixed` 为真、基线长度大于零、参与定向的卫星数大于零 |

当前 `position_fixed` 采用窄巷固定解判定；其他固定类型仍原样显示在
`position_type` 中，因此 `position_fixed=false` 不等于“没有位置解”。
正常固定航向通常可用，`heading_valid` 是额外检查异常报文。它表示本条
报文是否通过检查，不包含时效；使用时还要检查状态是否超过 2.5 秒未更新。
真北 `0°` 在定向有效时允许发布，无解时的占位 `0°` 会被拦截。

位置浮动、定向固定的字段组合示例（不是实测记录）：

```json
{
  "position_type": "NARROW_FLOAT",
  "position_fixed": false,
  "heading_type": "NARROW_INT",
  "heading_fixed": true,
  "heading_valid": true
}
```

完整正常报文还包含 `source`、`stamp`、`latitude`、`longitude`、
`baseline_m`、`heading_deg`、`pitch_deg`、`heading_satellites`。
其中 `heading_deg` 为设备定向角，不是 `/heading` 经安装偏角和 ENU 转换后的 yaw。
校验或解析失败时发布 `error`，并将定向有效性置为 false；其他字段可能缺失，
不能把缺失字段当作有效值。

### 定位保护、启动与续推

默认高精度模式首次初始化仍要求可靠位置和航向。位置通过 GGA `gps_qual=4`、协方差、
卫星数、HDOP 等条件判断，不能只看 `position_fixed`。初始化后，室外全系统默认允许合格的非 Fixed 位置慢速校正，详见下面的运行中策略；完全没有可信位置校正时沿用 FAST-LIO 续推，保留最近一次合格校正后的 100 米 / 600 秒限制。未加入“仅靠 GNSS 航向延长续推预算”的功能。

### 高精度与低精度启动开关

默认 `low_precision_startup=false`，保持原来的高精度启动条件：

```bash
roslaunch neupan_ros full_system_run.launch outdoor_mode:=true
```

需要固定航向配合较粗 GPS 起点启动时，显式开启：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true low_precision_startup:=true
```

该开关也可用于 `outdoor_global_only_test.launch`。它仅作用于默认的稳定定位
分支 `use_global_ekf=false`，不是全局 EKF 分支的启动参数行为。

低精度启动仍必须具备有效的位置解、固定且有效的双天线航向、新鲜且正常的
局部里程计；无位置解或仅有 heading 不允许启动。低精度位置允许 GGA quality
`1/2/4/5`，默认至少 6 颗卫星、HDOP 不超过 5、水平协方差不超过
100 m²。初始化保留原来的 2 秒稳定确认，候选地图平移相对均值的散布允许
到 2 米；航向稳定性和跳变检查不放宽。地图位置仍可能有数米甚至更大的偏差，
2 米散布条件不是绝对定位误差上限。

开关打开后优先尝试高精度初始化；质量或位置稳定性达不到高精度要求时才
采用低精度条件。低精度初始化取稳定窗口中的平均平移、有效定向建立
一次全局参考，随后进入 `LOCAL_DEAD_RECKONING`，全系统200 米 / 600 秒预算从初始化时开始，停车等待也计入。新的运行中非 Fixed 位置策略在独立条件通过后才纠偏及恢复预算，不沿用启动时宽松的 100 m² 条件；单点解不用于运行中校正。关闭运行中策略后则保持原来的冻结参考与续推预算。可靠 RTK 位置和航向恢复后，通过原有稳定窗口和平滑校正回到 `GPS_TRACKING`。后续目标点沿用当前参考，不会重新执行粗位置初始化。

`/gps_guard/status` 新增：

- `low_precision_startup_enabled`：开关状态。
- `initialization_precision`：实际首次初始化采用 `high` 或 `low`，尚未初始化为 null。
- `global_reference_precision`：初始参考为 `high` / `low`；运行中位置校正可为 `runtime_float` / `runtime_differential` / `runtime_fixed_position_only`，均不宣称完整 Fixed 位置+航向精度；恢复可靠 RTK 后为 `high`。
- `low_precision_limits`：本次运行的低精度检查阈值。

相关原因包括 `low_precision_stability_check` 和初始化瞬间的
`low_precision_initialized`；随后可能显示 GPS 协方差、GGA 质量或恢复等待原因。
这些字段随原有 GPS guard 状态写入日志。
阈值可通过 `global_odom_bridge` 的私有参数
`low_precision_max_covariance`、`low_precision_max_hdop`、
`low_precision_min_satellites`、`low_precision_consistency_m` 配置。
低精度模式要求定位保护、航向使用和定向固定检查均开启，冲突配置会明确报错。

固定航向改善初始方向可信度，不能保证局部规划不会异常，也不能消除粗位置
对云端经纬度目标、路网对齐和到达判断的误差。新增模式已通过消息级离线
测试和 launch 参数传递检查，仍需低速实车验证。

默认参数：`require_heading_fixed=true`、`solution_timeout_sec=2.5`。
可在 `g90_nmea.launch` 和 `wheeltec_dual_rtk_driver_nmea.launch` 中设置；
`global_odom_bridge` 也有同名私有参数。2.5 秒是定向状态的超时阈值，
不替代 `/heading` 自身的新鲜度检查；默认航向消息超时仍为 0.5 秒。
缺少 PVTSLNA 时默认停止发布航向，并阻止可靠 GPS 初始化或完整位置+航向校正；已初始化后的独立位置校正不依赖未固定或缺失的 GNSS 航向。

`/gps_guard/status` 的相关拒绝原因：

- `heading_solution_missing`：尚未收到定向解状态。
- `heading_solution_stale`：状态超时。
- `heading_solution_unfixed`：定向未固定或有效性检查失败。

实际 `reason` 还可能先报告位置或本地里程计异常，因为定位检查按顺序执行。

### 运行中不要求 Fixed 的位置校正（2026-10-16）

全系统默认 `gps_guard_allow_nonfixed_runtime:=true`，仅稳定定位分支 `use_global_ekf=false` 生效；原始 `navigation_core.launch` 和节点私有参数默认关闭，避免改变其他系统。可显式设置为 false 恢复原 Fixed 校正策略。启动门槛、室内定位、正常规划、掉头与相机靠右逻辑不变。

运行中接受 GGA `2/4/5` 的合格位置，拒绝 SINGLE、无解、未知协方差、无效坐标和旧数据。默认卫星≥8、HDOP≤2.5、水平协方差≤4 m²；GPS fix、投影里程计和 LIO 的源时间必须新鲜且匹配。保持原地图航向对齐，不采用未固定的 GNSS 航向，车辆朝向随 LIO 变化。

使用新 GPS 样本与 LIO 构成的地图平移候选，至少 8 个不同源时间样本、连续≥5 s，候选相对窗口中值的散布≤0.75 m；相邻候选跳变超过 1 m、样本中断超过 1 s 或来源类型变化时重新确认。候选与当前地图平移偏差超过 3 m 时拒绝，不自动拉回。通过后最多以 0.10 m/s 缓慢修正平移，残差≤0.5 m 才恢复续推预算，最多每个 5 s 窗口恢复一次。0.5 m 是候选一致性残差，不是对真实定位误差的保证；Float／差分精度仍可能明显低于 RTK。

`GPS_POSITION_TRACKING` 表示位置校正有效；`runtime_position` 记录 `enabled/active/reason/profile/samples/residual_m/budget_renewals/last_budget_renewal_sec` 和实际门槛。终端每 5 s 黄色提示该粗位置模式；JSONL 与原录包配置已有 `/gps_guard/status`，不需新增录制话题。没有可靠位置时，全系统重新累计200 m / 600 s，任一到限停车；LIO超时/跳变保护始终保留，不能靠旧报文或单帧Float重置预算。扩大续推距离不代表定位误差受控。

源时间保护对 Fixed、粗位置启动和运行中校正统一生效：检查 GPS fix、投影、GGA、LIO 的新鲜度、顺序与时间配对；使用 GNSS 航向时另查航向及解状态源时间。同一 GPS 源样本只处理一次，重复转发不能累计启动/恢复窗口、重复纠偏或恢复预算。Fixed 失效时先进入续推并启动预算，再等待配对；短暂配对等待保留已有窗口，但不新增样本或发放预算，持续失配仍受全系统200 m / 600 s限制。状态中的 `last_processed_gps_source_ns` 用于核对样本是否真的更新；[自测记录](videos/localization_runtime/20261016_nonfixed/summary.md)。

2026-10-17 补充安全信号断流保护：室外默认定位分支下，`control_mode_mux` 用单调接收时钟检查 `/localization_safe`，默认 1 秒未更新就令自动速度归零（再加正常调度下最多一个 20 Hz 发布周期，不是实测制动距离）。全系统参数 `localization_safety_timeout:=1.0` 可调；未收到、超时、明确不安全分别记为 `localization_safety_missing`、`localization_safety_timeout`、`localization_unsafe`，状态变化时终端黄色 WARN。重新收到新鲜 true 且自动指令未过期后恢复放行；手动接管、室内及原 `use_global_ekf` 分支行为不变，不新增 Fixed 要求。`/vehicle/control_mode/status` 每 0.5 秒更新有效安全状态、收到的原值、心跳年龄和时限；原 JSONL/录包已包含该话题，无需增加录包配置。这只检查安全消息是否持续到达，不等于修复 LIO 数据本身延迟或 ECOS 异常。

全系统可调参数为 `gps_guard_runtime_position_max_covariance`、`gps_guard_runtime_position_max_hdop`、`gps_guard_runtime_position_min_satellites`、`gps_guard_runtime_position_stable_sec`、`gps_guard_runtime_position_consistency_m`、`gps_guard_runtime_position_max_error_m` 和 `gps_guard_runtime_position_correction_rate`。这是低速封闭场地待实车验证的降级模式，不能当作 RTK 精度或完全无 GPS 无限续推。ECOS 求解异常未在本次修改中处理。

### 查看与验证

NUC 输入 `Neupan`、按 `e` 进入 ROS1 环境，启动全系统后查看：

```bash
rostopic echo /gnss/solution_status
rostopic echo /gnss/gpgga/gps_qual
rostopic echo /gps_guard/status
rostopic hz /heading
```

GGA 中 `gps_qual=4` 为位置固定解，`5` 为浮动解，`1` 为单点定位，`0` 为无定位。
无有效定向时 `/heading` 不发布，此时 `rostopic hz` 无数据是预期行为。
停止发布后不要把此前收到的航向缓存当作当前有效值。

重启系统后生效。`system_run_logger` 已将解状态保存为 `gnss_solution`、
`gnss_solution_age_sec`，导航事件上下文和 `/gps_guard/status` 也包含该信息。
现有 rosbag 配置尚未显式录制独立的 `/gnss/solution_status`；需要原始状态
话题回放时应将它加入所使用的录包配置。

验证已覆盖真实无解 PVTSLNA、位置/定向独立组合、CRC 错误、截断、非有限数值、
异常基线、有效零度航向和状态超时。5 项解析/驱动测试通过，定位保护检查通过；
实际设备无解时成功拦截占位零度航向。固定解正例使用合成报文，仍需室外实测。

# 十二、道路视觉与相机

当前主导航使用 MID360 避障、OSM 和 NeuPAN。GPU 板负责相机与 HybridNets 推理；NUC 接收视频和 JSON，可生成叠加图、输出 `/dev/video0` 并录包。雷达路缘与摄像头靠右控制均默认关闭；摄像头控制已提供显式开关，见下文“摄像头靠右控制”，尚未完成实车验收。

## 12.1 MID360 激光雷达路缘检测

路缘检测相关包：

| 路径 | 定位 | 是否直接用于当前 MID360 系统 |
|---|---|---|
| `src/lidar_curb_detection` | 上游固定扫描线算法参考源码 | 否；MID360 没有可直接套用的固定线束排列 |
| `src/lidar_curb_refinement` | MID360 高程栅格检测、误差滤波、路径修正 | 是 |

主要文件：

| 文件 | 作用 |
|---|---|
| `lidar_curb_refinement/scripts/lidar_curb_node.py` | 从右前方点云检测连续高度台阶，输出右侧路缘和横向误差 |
| `lidar_curb_refinement/scripts/path_refine_node.py` | 沿路径行驶方向的右法向量修正 OSM 路径 |
| `lidar_curb_refinement/scripts/curb_run_logger.py` | 持续写入路缘状态、误差、边缘点和路径概况 |
| `lidar_curb_refinement/launch/road_curb_debug.launch` | 单独检测及离线调试入口 |

主要话题：

| 话题 | 消息类型 | 作用 |
|---|---|---|
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | FAST-LIO 去畸变后的车体点云输入 |
| `/road_curb/right_edge` | `geometry_msgs/PointStamped` | 检测到的右侧路缘代表点 |
| `/road_curb/lateral_error` | `std_msgs/Float32` | 路径横向修正量 |
| `/road_curb/status` | `std_msgs/String` | JSON 格式的有效性、置信度和拒绝原因 |
| `/osm_global_path_curb_refined` | `nav_msgs/Path` | 供 NeuPAN 或直跟控制器使用的靠右路径 |

录包和 rosbag 离线回放命令见 [1.3](#13-rosbag-一键录制) 和 [1.4](#14-rosbag-离线路缘测试)。

## 12.2 USB Camera3、远程图像与 `/dev/video0`

相机内参、外参和 NUC 本地调试驱动位于：

```text
src/usb_camera/
```

主要结构：

```text
usb_camera/
├── config/
│   ├── camera.yaml
│   ├── usb_camera.yaml
│   └── livox_camera_extrinsics.yaml
├── launch/
│   ├── camera.launch
│   ├── calibrate.launch
│   └── livox_camera_tf.launch
├── scripts/
│   ├── rotate_image.py
│   └── calibrate_lidar_camera.py
├── README.md
└── EXTRINSIC_CALIBRATION_GUIDE.md
```

当前正式运行链路中，相机连接在 GPU 板上；NUC 不再用本地 `usb_cam_node` 作为主图像源：

```text
GPU 板 USB Camera3
   ↓ 旋转 180°、1920×1080
HybridNets + H.264/RTP
   ├─ UDP 5000：视频
   └─ UDP 6000：推理 JSON
                ↓
NUC hybridnets_receiver
   ├─ /camera/remote_image_raw
   ├─ /camera/remote_camera_info
   ├─ /hybridnets/debug_image
   └─ /dev/video0（虚拟摄像头）
```

远程图像参数：

| 参数 | 当前值 |
|---|---|
| NUC 虚拟设备 | `/dev/video0`，名称 `hybridnets_remote` |
| 分辨率 | `1920 × 1080` |
| 帧率 | `30 FPS` |
| `/dev/video0` 像素格式 | `YUYV` |
| 标定坐标系 | `camera_optical_frame` |

`/dev/video0` 是远程视频的虚拟摄像头；有设备节点不代表 GPU 板已发送画面。

启动远程接收并查看：

```bash
roslaunch hybridnets_receiver receiver.launch show_overlay:=true
ffplay -f v4l2 -i /dev/video0
```

OpenCV 可使用：

```python
cap = cv2.VideoCapture(0)
```

本地相机仅用于标定或单独调试，通常为 `/dev/video2`，勿与虚拟 `/dev/video0` 混用：

```bash
roslaunch usb_camera camera.launch device:=/dev/video2 preview:=true
```

本地调试链路才使用：

```text
/camera/image_raw_unrotated -> rotate_image.py -> /camera/image_raw
/camera/image_raw
/camera/camera_info
```

**坐标约定：** 内参、外参及推理像素均对应旋转 180° 后的 `1920×1080` 图像。使用 GPU 相机时勿再启动 NUC 本地相机。

## 12.3 相机内参标定

内参文件：`src/usb_camera/config/usb_camera.yaml`；分辨率：`1920×1080`。

相机矩阵与畸变参数：

```text
fx = 1102.837297249009
fy = 1104.805244636163
cx = 929.1071103499039
cy = 716.3749386027972
k1 = -0.3440482313019139
k2 =  0.08610560344908336
p1 = -0.003154023779031887
p2 = -0.004847752761592449
k3 =  0
```

标定启动：

```bash
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source devel/setup.bash
roslaunch usb_camera calibrate.launch
```

默认棋盘格为 `8×12` 个内部角点、方格边长 `0.020 m`。若 `8×12` 指方格数量，改用：

```bash
roslaunch usb_camera calibrate.launch size:=7x11 square:=0.020
```

ROS 默认保存至 `/home/nuc/.ros/camera_info/usb_camera.yaml`；确认后同步到项目内参文件。

## 12.4 MID360-Camera 外参标定

外参文件：`src/usb_camera/config/livox_camera_extrinsics.yaml`。

当前正式配置采用 2026-09-20 提供的最新 `T_cam_lidar`。它覆盖了旧的 `pose_01.bag` 联合结果，因此旧结果的约 `1 cm` RMSE 不能用于评价当前矩阵；配置中的 `joint_rmse_m` 为 `null`。

变换关系：`p_camera = R_camera_lidar × p_lidar + t_camera_lidar`。矩阵：

```text
R_camera_lidar =
[-0.031382438, -0.999488148, -0.006211597
 -0.013786343,  0.006646921, -0.999882871
  0.999412367, -0.031293127, -0.013987883]

t_camera_lidar =
[0.001691342, -0.080829955, -0.071189642]
```

ROS TF 发布其逆变换 `livox_frame -> camera_optical_frame`：

```text
translation = [0.0700865376, 0.0000000001, -0.0818057739]
quaternion_xyzw = [-0.4939530655, 0.5128394588, -0.5026797277, 0.4902235713]
```

启动：

```bash
roslaunch usb_camera livox_camera_tf.launch
```

检查：

```bash
rosrun tf tf_echo livox_frame camera_optical_frame
```

### 当前外参验收状态

外参已写入配置。视觉接入导航前，需用点云重投影检查：

1. 墙面点落在真实墙面区域；
2. 门框、桌沿、立柱等深度突变边缘与图像边缘一致；
3. 近处和远处均能对齐；
4. 多个独立场景有效，而不是只在标定板场景有效；
5. 静止正确后再检查运动中的时间同步误差。

如果静止场景对齐而运动场景出现整体错位，应优先检查时间同步，不要直接重新调空间外参。

`full_system_run.launch` 中 `base_link -> livox_frame = (0.10, 0.00, 0.35) m`，仍需以实车测量为准。

## 12.5 FAST-Calib 适配

上游包 `src/FAST-Calib`（ROS 包名 `fast_calib`）的本车适配文件：

```text
src/FAST-Calib/config/neupan_params.yaml
src/FAST-Calib/launch/neupan_calib.launch
src/FAST-Calib/launch/neupan_multi_calib.launch
```

`neupan_params.yaml` 已写入 USB Camera3 内参，并按 MID360 约 `2.2 m` 的采样密度调整聚类参数；默认 LiDAR 输入为 `/livox/lidar`。

单场景调用形式：

```bash
roslaunch fast_calib neupan_calib.launch \
  bag_path:=/path/to/calibration.bag \
  image_path:=/path/to/image.png \
  rviz:=true
```

另有 `usb_camera/scripts/calibrate_lidar_camera.py` 用于多静止场景联合求解；与上游 FAST-Calib 的结果需分别标注来源。

## 12.6 HybridNets：可行驶区域与车道线检测（已实现实时接收）

GPU 板通过独立网口发送 H.264/RTP 原视频和推理 JSON；NUC 本地生成调试叠加图，不接收第二路视频。

GPU 板启动：

```bash
conda deactivate
cd /home/p/lxw/lxwgdutcar_ai_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m colcon build --packages-select usb_camera --symlink-install
source install/setup.bash

ros2 launch usb_camera hybridnets_stream.launch.py \
  backend:=tensorrt \
  target_ip:=192.168.1.10 \
  video_port:=5000 \
  result_port:=6000 \
  bitrate:=8000000
```

只测接收时，先启动 NUC 接收器，再在 GPU 板**单次回放** MP4：

```bash
conda deactivate
cd /home/p/lxw/lxwgdutcar_ai_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch usb_camera hybridnets_video_replay.launch.py \
  video_path:=/home/p/lxw/lxwgdutcar_ai_ws/video/HybridNets/inputs/rosbag20260921_153216/rosbag20260921_153216_camera.mp4 \
  backend:=tensorrt \
  target_ip:=192.168.1.10 \
  video_port:=5000 \
  result_port:=6000 \
  bitrate:=8000000 \
  playback_rate:=1.0 \
  loop:=false
```

GPU 已生成完整检测与分割的 FP16 TensorRT 引擎，正常启动无需重新生成；ROS2 命令在退出 Conda 的终端执行，推理节点使用 GPU 原有的 `hybridnets` Conda 环境。修改源码后才需重新编译。

两条命令均用 `backend:=tensorrt` 启用加速；改为 `backend:=pytorch` 可切回原后端，省略参数也默认使用 PyTorch。GPU 端 2026-10-02 的 7 个关键帧测试约为 `14 Hz`，这是核心处理速度，不是 NUC 接收帧率。引擎路径为 GPU 工作区的 `src/HybridNets/weights/hybridnets_full_384x640_fp16.engine`；换 GPU 或 TensorRT 版本后需重新构建，步骤见 GPU 端 `src/usb_camera/TENSORRT_GUIDE.md`。

JSON 新增 `inference_backend` 和 `detections`，原视频端口、左右边界字段及 NUC 接收命令保持兼容；可检查 `inference_backend` 是否为 `tensorrt` 来确认后端。

实时相机与 MP4 回放二选一；回放画面不对应实时雷达，不能用于相机—雷达同步或融合验证。

要保存叠加画面，先在 NUC ROS1 环境启动接收和录制（不弹窗）：

```bash
roslaunch hybridnets_receiver receiver.launch \
  start_overlay:=true show_overlay:=false record_debug_mp4:=true
```

再启动 GPU 回放。首帧到达后写入 `/home/nuc/lxw/neupan_ws/videos/hybridnets_debug/`；断流约 3 秒后完成 MP4。若接收器已运行，勿重复启动，另开 NUC 终端执行：

```bash
rosrun hybridnets_receiver debug_mp4_recorder.py
```

同时保存同名 `YYYYMMDD_HHMMSS.mp4` 和 `.jsonl`（北京时间，同秒重名自动加序号）。JSONL 每行保存实际录入叠加视频的推理结果，包含 `mp4_frame_index`（从 0 开始）、`mp4_time_sec` 和匹配图像时间戳；补帧不重复写 JSON。默认生成 `20 FPS` MP4，并按图像时间戳补帧保持实际时长；可用 `debug_mp4_fps` 调整输出流畅度。完整接收 JSON、原视频和雷达数据仍可用 rosbag 保存。

GPU 板本机检查：

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /hybridnets/result_json --once
ros2 topic hz /hybridnets/drivable_mask
```

GPU 板查看叠加图：退出 Conda 后用 `rqt_image_view` 选择 `/hybridnets/overlay`。`/hybridnets/drivable_mask` 是黑白掩膜；`/camera/image_raw` 是旋转后的原图。

NUC 的 `/hybridnets/debug_image` 将 JSON 画在原图上：绿色为可行驶区，紫色/黄色为左右道路边界，不绘制车道线。NeuPAN 尚未使用该图或 `/hybridnets/result_json` 控车。

接收端会缓存图像，并用 `capture_timestamp_ns`、`source_age_ms` 和 RTP 延迟匹配推理帧；图中 `sync=...ms` 是候选帧时间差，不代表真实网络延迟。可用明显运动或闪光画面校准 `overlay_video_delay_sec`：

```bash
roslaunch hybridnets_receiver receiver.launch \
  start_overlay:=true overlay_video_delay_sec:=0.10
```

GPU 端应保证 JSON 的 `capture_timestamp_ns` 直接复制推理输入帧时间戳，并增加单调递增的 `frame_seq`，不要在推理完成时重新取时间。要做到严格逐帧一致，GPU 应在推理回调中对原输入帧生成 `/hybridnets/overlay`，或让 H.264 与 JSON 传输同一个可恢复的 `frame_seq`；普通 RTP 视频本身不能让 NUC 恢复应用层帧号。

当前 JSON 坐标属于 `rotated_image_pixels`：分辨率为 `1920×1080`，左上角为原点，`u/x` 向右、`v/y` 向下。主要字段包括：

| 字段 | 含义 |
|---|---|
| `capture_timestamp_ns` | GPU 相机采集时刻，纳秒 |
| `publish_timestamp_ns` | GPU 发送结果时刻，纳秒 |
| `frame_id` | 相机坐标系 |
| `image_width`, `image_height` | 必须匹配 `1920×1080` 内参 |
| `valid` | 是否找到有效可行驶区域 |
| `inference_ms`, `inference_hz` | 推理耗时与频率 |
| `source_age_ms` | 发送时图像年龄 |
| `road_polygon_px` | 道路外轮廓像素点 `[u,v]` |
| `left_boundary_px`, `right_boundary_px` | 左右道路边界，由近到远 |
| `left_boundary_segments_px`, `right_boundary_segments_px` | 左右边界连续段；绘图、拟合逐段处理，禁止跨缺口连线 |
| `left_valid_segments_px`, `right_valid_segments_px` | 通过独立质量检查的有效连续段；后续拟合只能使用这两个字段 |
| `left_segment_quality`, `right_segment_quality` | 各测量段的几何质量分数，与 `*_boundary_segments_px` 顺序对应 |
| `boundary_row_step_px` | 边界采样行间距；旧字段断线阈值为其 1.5 倍，缺省阈值 30 像素 |
| `lane_lines_px` | 多条车道线像素点 |

当 GPU 不做路口/场景分类时，`scene_type` 可固定为 `unknown`；NUC 仍接收并使用道路区域和左右边界，后续控制不依赖该字段。

GPU 当前输出 `normal` / `unknown`，场景字段不决定是否保留边界。`valid=true` 仅代表道路区域有效；使用某侧边界前必须分别检查 `left_boundary_valid` / `right_boundary_valid`，并结合接收端 `/hybridnets/valid` 检查时效。控制拟合逐段使用 `*_valid_segments_px`；`*_boundary_segments_px` 含未通过质量检查的测量段，只用于调试。NUC 叠加图以暗色细线画全部测量段，以亮色粗线画有效段；兼容旧平铺点列表时，按相邻点 y 差断线。录制 MP4 使用相同的分段叠加画面，JSONL 保留完整新字段。

这些是像素坐标，不能直接用于控制。当前 PyTorch 推理约 `2.7 Hz`。

## 12.7 GPU 板 → NUC 实时视觉传输（已实现）

nuc15 与 Jetson 当前使用 `192.168.1.0/24`，雷达口也使用该网段但属于另一条独立链路。已通过持久化主机路由区分 `.20/32`（Jetson 口）和 `.183/32`（雷达口），并设置接口 ARP 约束；配置见 [nuc15设备接入.md](nuc15设备接入.md)。

| 设备/接口 | 地址 | 用途 |
|---|---|---|
| GPU 板 `eno1` | `192.168.1.20/24` | ROS2 推理与发送 |
| nuc15 `enxec1ac30270b6`，连接名 `GPU板通信` | `192.168.1.10/24` | ROS1 接收，ASIX USB 千兆网卡 |
| NUC `enp86s0` | `192.168.1.100/24` | MID360 专用 |

NUC 主机检查网络（不要在 ROS1 容器内修改 NetworkManager）：

```bash
sudo nmcli connection up "GPU板通信"
ip -4 -brief addr
ip route get 192.168.1.20
ping 192.168.1.20
```

当前路由应显示接口 `enxec1ac30270b6`、源地址 `192.168.1.10`。nuc15 的上网接口为 `enxec1ac30270b0`（DHCP）；雷达与 Jetson 专用口不设置默认网关。

端口约定：

- UDP 5000：旋转 180° 后的原始相机 H.264/RTP；
- UDP 6000：一个数据报对应一个完整 HybridNets JSON；
- 不通过 ROS1/ROS2 bridge 传输图像和结果。

NUC 进入 ROS1 环境：

```text
Neupan
按 e
```

只测试接收链路：

```bash
roslaunch hybridnets_receiver receiver.launch show_overlay:=true
```

单独运行接收器时默认只接收原视频和 JSON，不生成 `/hybridnets/debug_image`：

```bash
roslaunch hybridnets_receiver receiver.launch show_overlay:=false
```

完整 NeuPAN 系统默认已经包含接收器，不要再单独启动第二个 `receiver.launch`：

```bash
roslaunch neupan_ros full_system_run.launch outdoor_mode:=true
```

查看接收和推理结果：

```bash
rostopic hz /camera/remote_image_raw
rostopic echo /hybridnets/valid
rostopic echo /hybridnets/inference_hz
rostopic echo /hybridnets/network_age_ms
rostopic echo -n 1 /hybridnets/result_json
```

全系统默认发布 `/hybridnets/debug_image`、不弹窗；可手动运行 `rqt_image_view /hybridnets/debug_image`。单独启动接收器时，`start_overlay:=true` 生成图像，`show_overlay:=true` 还会弹窗。

主要 ROS1 输出：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/camera/remote_image_raw` | `sensor_msgs/Image` | GPU 板原始画面 |
| `/camera/remote_camera_info` | `sensor_msgs/CameraInfo` | 当前相机内参 |
| `/hybridnets/result_json` | `std_msgs/String` | 校验后的完整结果 |
| `/hybridnets/valid` | `std_msgs/Bool` | 网络、时效与模型综合有效性 |
| `/hybridnets/inference_hz` | `std_msgs/Float32` | GPU 推理频率 |
| `/hybridnets/network_age_ms` | `std_msgs/Float32` | 两板对时后的结果年龄 |
| `/hybridnets/capture_header` | `std_msgs/Header` | GPU 采集时间戳 |
| `/hybridnets/timing_diagnostics` | `std_msgs/String` | 会话身份、帧确认、时效及拒绝原因；三份录包配置均记录 |
| `/hybridnets/debug_image` | `sensor_msgs/Image` | NUC 生成的道路/边界叠加图 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 接收健康状态 |

日志中的 `0.0.0.0:6000` 表示监听所有本地接口；`allowed_sender_ip: "192.168.1.20"` 仍限制发送方。

接收配置位于 `src/hybridnets_receiver/config/receiver.yaml`。收不到数据时，在 NUC 主机检查：

```bash
sudo tcpdump -ni enxec1ac30270b6 udp port 5000 or udp port 6000
```

结果超时 `0.8 s`、过期 `800 ms`、无效或格式不符时发布 `/hybridnets/valid=false`。两板需用 `chrony` 或 PTP 对时。

靠右控制模式按同会话连续3帧确认；epoch/生产进程变化后重新确认，退役会话、重复乱序及过期包不刷新活动心跳。路径端同时校验当前JSON自身的确认结果。当前GPU未发`producer_session_id`时兼容UDP源端口＋epoch；若重启时两者均复用则无法可靠区分，GPU应增加每进程唯一ID，NUC已支持。实际重启/跳时恢复及室外靠右仍需验收，详见 [待解决问题](PROJECT_ISSUES_CN.md#42-接收端会话已补齐动态恢复待验收)。

## 12.8 Vision + LiDAR 道路约束融合（待完成）

> 本节为后续算法规划，当前工程尚未实现视觉融合闭环。

目标是用视觉与 MID360 路缘共同约束 NeuPAN 路径。待完成：

1. 使用 `capture_timestamp_ns` 匹配最近的 Livox 点云；
2. 使用 `usb_camera.yaml` 内参和 `livox_camera_extrinsics.yaml` 外参把点云投影到图像；
3. 筛选落在 `road_polygon_px` 内的三维点；
4. 把这些点转换到 `base_link`，生成米制道路边界或局部栅格；
5. 再与 MID360 路缘结果交叉验证并修正参考路径。

先用 `hybridnets_data.yaml` 录包验证投影和时序，再低速实车测试；像素坐标不能直接控制转向。

建议形成以下回退逻辑：

```text
LiDAR 路缘可靠 + Vision 可靠
        -> 融合道路约束

LiDAR 路缘可靠 + Vision 不可靠
        -> LiDAR 路缘修正

LiDAR 路缘不可靠 + Vision 可靠
        -> Vision 可行驶区域修正

两者都不可靠或网络超时
        -> 不使用视觉修正，并按安全策略减速或停车
```

可以把两类感知都统一输出为：

```text
横向修正量 e
置信度 c
有效性 valid
```

融合权重和路径平滑应依据实车数据确定。

## 12.9 历史视觉与相机方案

以下内容仅作为历史调研保留，不代表当前主工程已经实现：

### HSV / Hough / CLRNet / `road_vision_refinement`

旧文档曾按以下思路设计视觉横向误差：

```text
校正 RGB 图像
-> HSV 白色/黄色候选
-> HoughLinesP + 位置/斜率/跨帧筛选
-> 车道线曲线与目标点
-> IPM 投影到 base_link 地面坐标
-> /road_vision/lateral_error
```

但在本次更新后的 `neupan_ws` 中，未发现 `src/road_vision_refinement` 包，因此不能把 `/road_vision/*` 相关话题写成“当前已实现”。若后续使用 HybridNets，应以新的实际包名、话题名和坐标转换流程重新填写。

### Orbbec Pro Plus

Orbbec 方案保留作为历史硬件调研：

- ROS1：`orbbec/ros_astra_camera`
- ROS2：`orbbec/OrbbecSDK_ROS2`

当前室外视觉主线已经转为 USB Camera3，因此新功能优先维护 `src/usb_camera`，避免两套相机驱动和标定参数混用。

# 十三、VLM 与世界模型

本章为后续多模态与世界模型调研，不参与当前室外导航主链路。


## 13.1 Jetson Orin NX Super

### 环境配置与编译

```bash
cd /home/p/lxw/lxwgdutcar_ai_ws
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:/bin:$PATH
colcon build
```

### qwen3-vl-4b

Qwen3-VL-4B-Instruct-AWQ-4bit：[Qwen3 VL 4B | Jetson AI Lab](https://www.jetson-ai-lab.com/models/qwen3-vl-4b/)

**vLLM 负责把模型跑起来。 Docker 负责把运行环境打包起来。**

#### 网络问题

docker挂梯子

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d

sudo tee /etc/systemd/system/docker.service.d/proxy.conf >/dev/null <<'EOF'
[Service]
Environment="HTTP_PROXY=http://127.0.0.1:7899"
Environment="HTTPS_PROXY=http://127.0.0.1:7899"
Environment="http_proxy=http://127.0.0.1:7899"
Environment="https_proxy=http://127.0.0.1:7899"
Environment="NO_PROXY=localhost,127.0.0.1"
EOF

sudo systemctl daemon-reload
sudo systemctl restart docker
```

#### 官方下载模型/运行模型

```bash
# Serve command
sudo docker run -it --rm --pull always \
--runtime=nvidia --network host \
ghcr.io/nvidia-ai-iot/vllm:latest-jetson-orin \
vllm serve cpatonn/Qwen3-VL-4B-Instruct-AWQ-4bit --gpu-memory-utilization 0.8
```

```bash
sudo docker run -it --rm --pull always \
  --runtime=nvidia \
  --network host \
  -e HTTP_PROXY=http://127.0.0.1:7899 \
  -e HTTPS_PROXY=http://127.0.0.1:7899 \
  -e http_proxy=http://127.0.0.1:7899 \
  -e https_proxy=http://127.0.0.1:7899 \
  -e HF_HOME=/root/.cache/huggingface \
  -v /home/p/.cache/huggingface:/root/.cache/huggingface \
  ghcr.io/nvidia-ai-iot/vllm:latest-jetson-orin \
  vllm serve cpatonn/Qwen3-VL-4B-Instruct-AWQ-4bit \
  --gpu-memory-utilization 0.6 \
  --max-model-len 4096
```

显存不够调整：

```bash
sudo docker run -d \
  --name qwen3-vl-4b \
  --restart unless-stopped \
  --runtime=nvidia \
  --network host \
  --ipc=host \
  -v /home/p/lxw/models/Qwen3-VL-4B-Instruct-AWQ-4bit:/model:ro \
  ghcr.io/nvidia-ai-iot/vllm:latest-jetson-orin \
  vllm serve /model \
    --served-model-name cpatonn/Qwen3-VL-4B-Instruct-AWQ-4bit \
    --gpu-memory-utilization 0.49 \
    --kv-cache-memory-bytes 268435456 \
    --max-model-len 1024 \
    --max-num-batched-tokens 1024 \
    --max-num-seqs 1 \
    --mm-processor-cache-gb 0 \
    --limit-mm-per-prompt '{"image":1,"video":0}' \
    --skip-mm-profiling \
    --enforce-eager
```

```bash
#启动：
/home/p/lxw/qwen3-vl.sh start
#停止：
/home/p/lxw/qwen3-vl.sh stop
#重启：
/home/p/lxw/qwen3-vl.sh restart
#查看状态：
/home/p/lxw/qwen3-vl.sh status
#查看日志：
/home/p/lxw/qwen3-vl.sh logs
#删除旧容器并按正确参数重新创建：
/home/p/lxw/qwen3-vl.sh recreate
#也可以直接使用 Docker 名称：
sudo docker stop qwen3-vl-4b
sudo docker start qwen3-vl-4b
sudo docker restart qwen3-vl-4b
sudo docker logs -f qwen3-vl-4b
```

#### 测试模型

```bash
curl -s http://127.0.0.1:8000/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "cpatonn/Qwen3-VL-4B-Instruct-AWQ-4bit",
    "messages": [
      {
        "role": "user",
        "content": "你好，请用中文介绍一下你自己。"
      }
    ],
    "max_tokens": 128,
    "temperature": 0
  }' | python3 -m json.tool
```



## 13.2 世界模型

RSSM+雷达encoder+neupan

## 摄像头靠右控制

室内无明显道路边界时，GPU也应持续发送JSON，只是`valid=false`；不能把`waiting_for_packet`当作正常无边界。10月21日测试包缺原视频、相机信息与JSON，靠右未生效；NUC已增加UDP到包数、来源拒绝数和实际来源诊断，GPU图像/发送链路仍待恢复验证。已有全系统运行时直接查看，不要重复启动接收器：

```bash
rostopic echo -n 1 /hybridnets/timing_diagnostics
```

`event=packet`说明JSON通过结构解析进入接收链路，`receiver_valid`说明接收时效/有效性检查结果；靠右是否实际使用还需看`/camera_right/status`的`active`。`sender_rejected`说明来源IP不符；`health`且收到数据报数为0说明尚无包进入该监听端口。未验证接收与有效边界前不做靠右实车控制。

普通室外和靠右控制仍使用最大50 cm参考采样（`outdoor_reference_spacing_m=0.50`），路线几何/终点不变。2026-10-27修正：1.5 m/75°阈值只作偏差诊断和低速预测请求（上限0.30 m/s），不再单凭起步约1.53 m偏差或等待时间永久锁停；输出仍须完整车体/时效/适用相机授权复查。扫描采用8帧有界同刻TF重试队列，保留原测量及接收时刻，有效旧快照不因新帧等TF被作废，真实超时仍停车；完整输入采用一个恢复窗口。预测碰撞先评估两档重积分降速方案，均不合格则零速并继续原任务重算；非有限或未知内部故障仍保持并需检查复位。准备后重新绑定求解位姿，不能给旧结果换时间戳。运行日志`timing`/扫描健康`capture_tf`提供阶段耗时、输入年龄、队列和原TF错误；相机`inference_diagnostics.osm_guidance_context`及观察/跳过点数解释路口优先，不再只显示确认中。GNSS串口统计仍只写日志。见[离线报告](videos/neupan_startup_recovery_20261027/report.md)，不是实车闭环许可；需重启原规划及靠右节点加载，GPU/室内规划不变。

独立节点 `osm_map_handler/camera_right_path.py` 使用道路段校正缓存，目标是相机光学中心横向距右边界约 2 m（不是三维斜距，也不是车轮距离）。它只修正全局参考路径，不改原 OSM 文件、不直接发布速度；速度和转角仍由 NeuPAN 计算，雷达避障、定位保护、指令超时和人工接管保留。默认关闭视觉控制。

代码统一归属 [`src/osm_map_handler`](src/osm_map_handler/README.md)：OSM A*、返程连接、相机路径修正与路径选择策略集中管理，公共 Python 模块以 `osm_` 前缀平铺在 `src/`，不套同名文件夹。`neupan_ros` 保留局部优化、控制接入与独立心跳监督。室外掉头适配也在该包，通过独立 worker 调用 `hybrid_astar` 的共享 C++ 核心，不复制第二套搜索算法。全系统命令和话题不变，单独启动相机路径用 `roslaunch osm_map_handler camera_right_path.launch`。

### 启动命令

进入 `Neupan` 后按 `e` 进入 ROS1，GPU 端同时启动实时摄像头推流与 JSON 发送。NUC 启用实际靠右控制：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp86s0 192.168.1.100 netmask 255.255.255.0

roslaunch neupan_ros full_system_run.launch outdoor_mode:=true \
  enable_camera_right_path:=true enable_camera_right_control:=true \
  camera_right_strategy:=segment_reference camera_right_camera_clearance_m:=2.0 \
  camera_right_clearance_min_m:=1.0 camera_right_clearance_max_m:=3.0 \
  camera_right_base_link_height_m:=0.20 \
  enable_hybridnets_overlay:=false enable_rviz:=false
```

首次建议先用影子模式：只计算视觉修正路线，不交给 NeuPAN 控制；车辆仍可能按原 OSM 路线行驶，并非停车模式。在上述 ROS1 环境中运行（已有全系统时先停止旧实例）：

```bash
roslaunch neupan_ros full_system_run.launch outdoor_mode:=true \
  enable_camera_right_path:=true enable_camera_right_control:=false \
  camera_right_strategy:=segment_reference camera_right_camera_clearance_m:=2.0 \
  camera_right_clearance_min_m:=1.0 camera_right_clearance_max_m:=3.0 \
  camera_right_base_link_height_m:=0.20 \
  enable_hybridnets_overlay:=false enable_rviz:=false
```

另开 ROS1 终端查看状态：

```bash
rostopic echo /camera_right/status
```

`active: true` 只表示产生视觉候选，不保证消费端已授权或实车间距达标；需同时核对`/neupan/camera_path_guard`和`/neupan/runtime`。`guidance_source: osm` 表示使用原路线，`reason`给出回退/停车原因。影子路径为`/camera_right/reference_path`，启用控制后NeuPAN的`/initial_path`接收`/camera_right/control_path`。无需开启绘图或RViz，不要同时启用`enable_lidar_curb_control`。

原视频由宿主机`neupan-remote-video.service`开机独立接收，Master由`neupan-ros-master.service`维持；退出NeuPAN不关闭原视频。整车默认`enable_hybridnets_video:=false`，`enable_hybridnets_receiver:=true`仍接收JSON。独立服务不启动推理处理、靠右节点、底盘或规划，GPU命令不变；`receiver.launch`默认也不重复启动视频。安装、停用和故障日志见[视频服务说明](src/hybridnets_receiver/README.md)。

### 核心条件与回退

**2026-10-28当前默认：`segment_reference`粗略稳定参考，约1–3 m内不追2 m。** 参考冻结在OSM的map坐标中；道路观测证据独立保存在odom、随对齐一致重投影。连续确认偏出范围后只生成一次满足转弯/空间变化率限制的过渡，未驶完入段时不因仍未到目标而逐帧改路。路口/转弯评估平顺回OSM，新任务重新评估。普通视觉断流/静态缓存失效不新增危险锁存，保留地图建议不代表续长旧观测或证明未观测道路安全。真实边界危险、定位/扫描时效、完整扫掠及恢复复位不变。以下10月27日合同/逐周期变形说明只适用于兼容`segment_cache`，不能套用为当前普通参考必须满足的许可条件。

三项复核已修：采样间隔与年龄独立，当前1Hz轮里程计仅记区间均速，不当瞬时约束；反馈不足不新增停车。靠右按当前进度/方向选择前方约2–8m证据，不用大量车后缓存压过当前路段，危险检查不缩小。拒绝证据逐轮清空并绑定`cycle_id`，文件名增加轮次，缺数据记`missing_fields`，不取上一轮轨迹。Noetic完整回归663通过、8可选跳过，两个功能包构建通过；需重启原节点生效，仍待实车验收。[本轮报告](videos/neupan_review_three_fixes_20261028/report.md)。

室外规划加入渐进限速保持、首步命令连续性及合格高频位置差分估计，发布前再检查；当前1Hz轮里程计只记录区间均速，真实转角编码器尚未接通，`/odom.twist`固定零或指令回显不能当实际反馈。工作区适配层在PAN每次迭代对完整扫描作车体几何选点，成对重排DUNE特征/障碍点；外部库文件、模型及目标函数未改。两档降速失败最多一次真实重求解。真实旧危险只在连续运动、当前扫描、剩余路线及最终预测都证明不再进入时临时排除，记录保留，返回/失效重新阻塞。拒绝证据写`tracker_logs/neupan_candidate_会话_轮次_序号.json/.npz`，上限64份/会话、1份/秒、队列2份，包含选点、反馈和轮次绑定。[当前状态](PROJECT_ISSUES_CN.md)。

10月28日ROS控制衔接：`neupan_execution.py`只管理同一份原生解的版本、期限、进度和发布，不增加预测器或规划器。求解阻塞时独立线程约10Hz复核最新扫描/位姿并推进剩余控制，不反复重发首步或续期旧解；任务变化、人工切换和显式停车均可撤销。本次复查过期最多完整重绑重查一次；两次仍过期而当前输入健康时可零速立即重算，但不误加断流恢复等待。真实断流保留原窗口。可选远端冲突例外已用含正倒车及转角执行不确定性的方向性占用替代全向圆域，必须有完整实测制动/延迟/转向边界；目前未填，不能用测试夹具上线，整段合格候选不新增门槛。原0.5s超时、原生停车及内部故障保护不放宽。原`/neupan/runtime`及JSONL增加重绑次数/年龄、冲突分类和`reference_receipt`路径装载回执，区分生产、消费与实际跟踪。需重启原规划节点，GPU不改；完整结果和仍待实测项只维护在[问题文档](PROJECT_ISSUES_CN.md)。

检查顺序补充：准备阶段即使耗时550 ms，只要队列里已有可转换的合格新扫描，会在健康检查和输入恢复判断前先刷新扫描及位姿，不因入口旧快照额外停车；求解前同快照绑定和发布最终时效检查保留。原0.5 s时效、恢复窗口、独立故障保持不变。需重启原规划节点加载；[验证报告](videos/neupan_precheck_order_20261027/report.md)不代表动态闭环已验收。

空扫描/扫描绑定补充（2026-10-27）：合法无障碍集通过ROS适配层转为实际库支持的`None`，退役上一帧障碍最小距离/显示点缓存，完整安全扫描及原时间仍保留；NaN、非法扫描及真实断流不作无障碍。准备结束后再刷新队列，从同一快照绑定规划点和完整扫描，再读取求解位姿。`timing.solve_input_ages`与`solve_queued_frames`给出求解绑定状态，最终时效检查不换扫描给旧结果续期。启动命令及0.5 s门限不变，重启原规划节点加载；[验证报告](videos/neupan_scan_binding_20261027/report.md)不替代实车验收。

2026-10-27发布协议：授权版本绑定路线、路径摘要及执行限制，普通心跳续期不替换授权。发布前以最新限速/道路检查真实预测，冲突先零速、下周期重算；重型检查之后，在短锁内复核版本、实际扫描与同历元位姿年龄、授权期限并发布，不借新扫描给旧计算续期。普通缓存失效保留固定odom复查锚点，但不延长旧观测；新鲜观测覆盖原范围后仅允许规划，实际预测及最终时效通过才确认解除。未知范围须人工核查。状态新增`recovery_pending`及已检查授权身份，运行日志区分求解授权和发布检查授权；[离线报告](videos/neupan_publication_fix_20261027/report.md)不替代实车验收。需重启规划与靠右节点，GPU命令不变。

- 新校正需定位安全、JSON和接收器valid信号新鲜。按区段保留真实左约束；左边界短或不交叠的部分可按单右边界处理，但须有真实可行驶多边形，不能伪造左侧或丢弃已知窄路冲突。当前使用segment_reference，realtime仅作影子对照；图像保持标定时1920×1080旋转后坐标。
- GPU/NUC时钟需同步，采集时间不可改成接收时间。segment_reference及兼容segment_cache新观测准入仍为800 ms采集龄/500 ms源龄，未来容差50 ms；30 s静态缓存不是新观测，冻结参考不续期视觉证据。接收端门控独立。
- 每个路线点需3个不同帧的一致观察，静态道路缓存最多30 s；过期不能获新视觉执行合同。新路线/定位异常清理普通缓存，独立真实危险不丢失。每轮绑定一份TF样本、按TF测量时间计算对齐变化率；约2 s处理间隔不再单独把3.6 cm平滑变化判跳变。
- 普通参考建议不因相机心跳丢失锁停；原路线不存在则停。已知真实危险不因断流、换任务编号或新候选自动解锁；相应恢复合同仍需消费者实际预测复核。状态看`/neupan/camera_path_guard`，停车原因看`/neupan/runtime.stop_reasons`；mux/定位/MCU保护独立保留。
- 相机中心间距约1–3 m内保持，不排序追求最近2 m；带0.10 m判断滞回，偏出需连续不同帧确认。相对OSM横移最多1.5 m，默认5 Hz观察，不按每帧几厘米移动参考。空间过渡按8/12/16/24/32 m候选完整检查并保持，长度不可行则保留合格参考，真实危险另行处理。
- 只有显式视觉执行能力/真实危险恢复才使用0.5 s合同及观测终点限制；有效期不能越过其依赖证据TTL。普通地图建议不受该终点约束，但也不保证未观测区域可通行；后续持续由定位、扫描和规划检查。

OSM优先处理路口/转弯：仍读取同一地图并检查前方约8 m、约20°路线变化；停止新增不相干靠右段并评估空间衔接，不强制2 m。暂时无边界可以保持普通地图建议，不能把旧观测改成新证据。真实危险不因该策略自动解除。

返程掉头（2026-10-16）：室外 OSM 直连默认启用 `enable_outdoor_start_connector:=true`、`outdoor_connection_planner:=hybrid_astar`。只有新路线接入处的行驶方向与车头夹角 ≥135°、需要掉头时，才用共享 C++ Hybrid A* 在 16×16 m、0.1 m 分辨率的局部图中搜索仅前进连接段，再接回原路线并保留终点。距离按连续路线段计算，仅作诊断，不再触发搜索。其他路线保持原 NeuPAN 跟踪、局部避障及扫描/定位保护，不新增距离、道路或掉头观测准入门槛；普通起步、同向偏移、90° 转弯不受影响。已移除先前的 20°/2 m 普通接回扩展及两个参数。掉头搜索预算 1 s、最多 12000 次迭代；等待期间停车，返回后用最新且采集 TF 对齐的扫描复查完整路径。正常路段仍走原 OSM A*，不是整段栅格重规划；不要打开旧 `outdoor_use_hybrid_astar` 分支。旧方案对照可设置 `outdoor_connection_planner:=dubins`，失败不会自动切换方案。

转弯半径同时满足规划配置与轴距/最大转角，当前至少 1.85 m；检查实际车体加 0.12 m 余量、OSM 道路范围、单行方向及雷达观测。道路宽度优先取 `width`，其次 `lanes×3 m`，缺失时用 `osm_default_road_width`；估计宽度不能替代实测。

不能连接时保持零速度，绝不把原来的反向参考线直接交给求解器。雷达 `inf/NaN`、盲区和缺少历史 TF 不算可通行空间；等待新鲜观测最多 5 s，随后或几何检查失败时拒绝该路线，需在更宽、已确认安全的区域重新下发目标。没有新增自动倒车/退车脱困，NeuPAN 原有倒车机制和室内默认搜索配置保留；共享核心补上迭代耗尽不返回半条路径、越界及地图未就绪保护。连接段完成前忽略同一目标的摄像头修正/原路线切换，避免覆盖掉头段；完成后恢复靠右更新。

2026-10-16 兼容近场过滤：原雷达安装外参、FAST-LIO `blind=0.5 m`、扫描 `range_min=0.4 m` 均不改。只调整掉头准入检查：当前实际车身占用区免查回波，未来车身进入的车外区域仍须观测；额外 12 cm 缓冲区继续拦截已知障碍，但落在近场过滤圆内的缓冲采样点可保持“未验证”，不当成空地，也不代表完整余量已验证。终端黄色提示，`admission.accepted_scan_check.unverified_near_margin_points` 记录点数。车身本体进入近场盲区、外侧无回波或道路空间不足仍停车；采集时刻 TF 最多缓存 8 帧短队列重试，观测有效期仍是 0.5 s，不改时间戳、不使用最新 TF 替代。

状态及拒绝原因（系统 JSONL 与三套录包配置均已记录）：

```bash
rostopic echo /neupan/route_connection
```

`searching` 表示停车搜索，`connecting` 执行连接段，`following_route` 正常跟踪，`blocked` 表示该路线被拒绝。`reason: normal_route_tracking` 表示保留原跟踪、不启动掉头搜索；`connection_kind: uturn`、`uturn_required: true` 表示该新路线触发掉头，角度阈值见 `uturn_heading_threshold_deg`。`start_gap_m` 为连续路线距离，`nearest_vertex_gap_m` 为顶点距离，仅用于排查。`admission` 是准入快照，不是实时避障结果；`search` 记录算法、耗时、展开节点等统计，`scan_stamp_sec` 为初始扫描时间，`completion_scan_stamp_sec` 为返回后复查时间。Hybrid A* 的 `road_rejected` / `scan_rejected` 按拒绝的采样姿态计数，不是整条路线数；扫描原因区分近场盲区、车身未知区、障碍/遮挡及缓冲区未知。`scan_transform_state` 给出采集 TF 状态。实际参考路线看 `/neupan_initial_path`，原路线仍为 `/osm_global_path`。旧退出日志缺少异常堆栈，不能据此宣称已证明或消除了当时退出的直接原因。

已通过几何/路径接入回归与隔离 ROS1 假传感器测试；使用当前模型进行 230 步运动学闭环仿真，完成前进掉头并接回返程路线，未启动车辆。仿真仍有求解精度告警 `Solved/Inaccurate`，结果均为有限值；实车效果、坡道及旧数值异常仍需新日志验证。

地面按 `base_link` 下方 0.20 m 计算（`camera_right_base_link_height_m:=0.20`，默认值），相机离地约 0.468 m；不修改标定外参。视频只投影近处合理视场内的连续路线，排除远处侧向路段，不跨缺口连线；实际导航仍保留完整路线。修正视频：[comparison.mp4](rosbag_videos/rosbag20261008_155045/right_driving_2m_ground20_fixed/comparison.mp4)。离线缓存修正 45/982 个样本，未应用全部实时保护，不代表实车控制已验收。

### 验证与录制

室外往返、窄路停车、观测不足和转弯区障碍的独立 Gazebo 仿真见 [仿真说明](src/neupan_ros/example/lxw_car_simulation/README.md#室外返程测试)。按实车安装及过滤参数生成点云并调用生产扫描节点，可用私有 master 自动测试，视频、日志及 rosbag 统一保存到 `videos/outdoor_simulation/日期_时间/`，不启动实车硬件。

2026-10-16 共享 Hybrid A* 四场景均通过：[结果与录像](videos/outdoor_simulation/20261016_113623/summary.md)。宽路完成前进掉头返程，终点误差约 0.488 m（阈值 0.5 m），搜索约 26 ms；窄路/未知回波/实体障碍均拒绝并持续零速度。旧 Dubins 对照保留在 [092222 报告](videos/outdoor_simulation/20261016_092222/summary.md)。原实车雷达、标定与室外模型配置未修改；不代表最小实车掉头宽度或实车控制验收。

同日补充 [21 次同起点窄路对照](videos/outdoor_simulation/20261016_142251_width_comparison/comparison.md)：两者在 9 m 道路均 2/2 完成。新版 8.75 m 仅 1/2 完成（成功时余量约 1.7 cm，另一次包络越界约 1.45 cm），8.5 m 两次均越界；原曲线在这些宽度拒绝并停车。当前没有可靠的更窄实控优势，需先处理紧弯跟踪误差与安全余量，不能按偶发成功宽度进行实车极限掉头。测试只扩展仿真入口，未改变生产规划算法；临时文件及重复录像已清理。

几何、时间对齐、心跳保护、长路线进度保留及异常诊断已通过单元与隔离 ROS1 故障验证；默认 3 s 心跳超时已验证。尚未完成实车控制验收，平地模型尚不补偿坡道与车身俯仰。先用实时数据完成影子验证，再在封闭场地低速、可随时人工接管的条件下测试控制。

同日按“仅掉头使用混合 A*”收紧触发条件：[验证报告](videos/outdoor_simulation/20261016_normal_rejoin_fix/summary.md)。101 项回归通过，15:37 日志中的四次目标保持原路线跟踪；隔离 ROS1 验证 3 m 偏移、90° 转弯不进入搜索，反向且观测不足仍停车。没有原始扫描，不能将接入回放等同实车避障验收。

`src/vehicle_bag_recorder/config/hybridnets_data.yaml` 已包含 `/camera_right/reference_path`、`/camera_right/control_path`、`/camera_right/status`，用于后续 bag 检查；旧 bag 超过时效门槛，不能将离线结果当作实时控制验收。

之前三个独立验证脚本已清理；返程衔接回归集中在 `src/Test/tests/test_outdoor_start_connection.py`，原有电机、开环运动及录制工具保留。bag 视频与外参验证统一使用 `src/usb_camera/scripts/validate_driving_extrinsics.py`。自维护功能包按 [功能包分类](src/README.md) 区分核心模块、可执行脚本和测试，不按 Python/C++ 一刀切；原启动命令与控制参数不变。
