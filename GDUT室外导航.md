# GDUT 小车室外导航开发与运行手册

> 工作区：`/home/nuc/lxw/neupan_ws`<br>
> 当前主系统：ROS1 Noetic + Livox MID360 + FAST-LIO + GPS/RTK + OSM A* + NeuPAN<br>
> 本文开头是日常运行速查；后半部分保留完整的环境部署、建图、仿真和算法调研记录。

## 目录

- [一、日常运行速查](#一日常运行速查)
  - [编译与环境准备](#11-编译与环境准备)
  - [三种室外导航模式](#12-三种室外导航模式)
  - [MID360 右侧路缘检测与靠右行驶](#13-mid360-右侧路缘检测与靠右行驶)
  - [Rosbag 一键录制](#14-rosbag-一键录制)
  - [Rosbag 离线路缘测试](#15-rosbag-离线路缘测试)
  - [实车运行检查](#16-实车运行检查)
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
- [十二、道路视觉与相机](#十二道路视觉与相机)
- [十三、VLM 与世界模型](#十三vlm-与世界模型)

# 一、日常运行速查

## 1.1 编译与环境准备

以下命令都在 ROS1 Noetic 环境中执行：

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
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
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

## 1.3 MID360 右侧路缘检测与靠右行驶

### 首次依赖检查

`tf2_sensor_msgs` 依赖系统 Python 的 `PyKDL`。可以正常执行 `conda activate lxw` 运行
NeuPAN；路缘 launch 使用 `launch-prefix=/usr/bin/python3`，三个路缘节点会单独使用系统
Python，不受当前 Conda 环境影响。首次在 ROS1 Noetic 容器执行：

```bash
sudo apt update
sudo apt install python3-pykdl ros-noetic-tf2-sensor-msgs
/usr/bin/python3 -c "import PyKDL, rospy, tf2_sensor_msgs; print('curb ROS Python OK')"
cd /home/nuc/lxw/neupan_ws
catkin_make
source devel/setup.bash
```

如果仍出现 `ModuleNotFoundError: PyKDL`，检查实际解释器：

```bash
head -1 /home/nuc/lxw/neupan_ws/src/lidar_curb_refinement/scripts/lidar_curb_node.py
/usr/bin/python3 -c "import PyKDL; print(PyKDL.__file__)"
```

参考源码位于 `src/lidar_curb_detection`。原算法依赖固定扫描线编号，适合 Velodyne 等机械式
雷达；MID360 使用非重复扫描，因此当前系统实际运行 `src/lidar_curb_refinement`，在 FAST-LIO
去畸变后的 `/cloud_registered_body` 上构建高程栅格并寻找车体右前方连续高度突变。

处理链路：

```text
/cloud_registered_body
  -> lidar_curb_node.py
  -> /road_curb/lateral_error
  -> path_refine_node.py + /osm_global_path
  -> /osm_global_path_curb_refined
  -> NeuPAN 或 global_path_tracker
```

`base_link` 中 x 向前、y 向左，右侧路缘 y 为负。路径修正使用路径行进方向的右法向量，车辆
沿同一条道路反向行驶时，右侧也会随行驶方向反转，不会固定向地图某个方向偏移。

### 第一步：仅启动检测，不接管车辆路径

建议先在安全路段观察和标定：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true \
  enable_lidar_curb:=true \
  enable_lidar_curb_control:=false
```

也可以只启动路缘模块，用于 rosbag 离线测试：

```bash
roslaunch lidar_curb_refinement road_curb_debug.launch \
  start_path_refine:=false \
  cloud_topic:=/cloud_registered_body
```

#### 自动日志（默认，无需 rostopic）

只要启用了路缘检测，系统就会同时运行 `curb_run_logger.py`，以 10 Hz 持续写入：

```text
/home/nuc/lxw/neupan_ws/tracker_logs/lidar_curb_YYYYMMDD_HHMMSS.jsonl
```

日志包含 `/road_curb/status`、横向误差、右侧边缘点、原始路径及修正路径概况。运行中查看最新
日志可以使用：

```bash
tail -f "$(ls -1t /home/nuc/lxw/neupan_ws/tracker_logs/lidar_curb_*.jsonl | head -1)"
```

按 `Ctrl+C` 结束 launch 时，日志会写入汇总并执行 `flush + fsync` 后关闭，不需要另外启动
`rostopic echo`。默认单文件上限为 256 MB，文件名和 `local_time` 明确使用北京时间 UTC+8，
不受 Docker 内部 UTC 时区影响。

项目自定义日志已统一使用北京时间 UTC+8，包括综合导航日志 `system_run_*.jsonl`、路缘日志
`lidar_curb_*.jsonl`、开环标定日志以及 rosbag 文件名和 JSON 清单。日志中保留的 `ros_time`、
`stamp` 和传感器消息时间戳不做时区换算，用于保证多传感器同步。

`/road_curb/status` 中：

- `valid=true`：已经连续确认右侧路缘；
- `right_distance_m`：车辆原点到右侧路缘的距离；
- `lateral_error_m`：建议路径横移量，正值向车辆右侧，负值向左；
- `reason`：无有效结果时的原因，例如点数不足、TF 不可用或路缘跳变。

默认期望右侧净距为 `0.55 m`。需要修改时：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true \
  enable_lidar_curb:=true \
  enable_lidar_curb_control:=false \
  lidar_curb_desired_clearance_m:=0.60
```

### 第二步：低速确认修正路径方向

单独输出修正路径供 RViz 对比：

```bash
roslaunch lidar_curb_refinement road_curb_debug.launch \
  start_path_refine:=true \
  input_path_topic:=/osm_global_path \
  output_path_topic:=/osm_global_path_curb_refined
```

必须确认以下项目后才能接管控制：

1. 正向和反向行驶时，修正路径都位于各自行驶方向右侧；
2. 路缘丢失后修正量平滑回到原始 OSM 路径；
3. 停放车辆、草坪、排水沟不会持续产生有效假路缘；
4. 横移不超过默认 `max_correction_m=0.45 m`；
5. 实际右侧净距小于目标值时，路径向左远离路缘，而不是继续向右。

### 第三步：启用靠右闭环

NeuPAN 完整系统：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true \
  enable_lidar_curb:=true \
  enable_lidar_curb_control:=true
```

全局路径直接跟踪测试：

```bash
roslaunch neupan_ros outdoor_global_only_test.launch \
  enable_lidar_curb:=true \
  enable_lidar_curb_control:=true \
  direct_tracker_speed:=0.30
```

两个开关必须同时为 `true`。只开启 `enable_lidar_curb_control` 时没有修正路径发布者，控制器不
会收到路径，这是有意设置的安全保护。

路缘日志默认开启。需要关闭或更换目录时：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true enable_lidar_curb:=true \
  enable_lidar_curb_log:=false

roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true enable_lidar_curb:=true \
  lidar_curb_log_dir:=/media/usb/tracker_logs
```

## 1.4 Rosbag 一键录制

通用录包功能包位于 `src/vehicle_bag_recorder`。它不绑定某个检测或导航算法，通过 YAML
管理话题，并调用 ROS 官方 `rosbag record` 保留原始消息类型和时间戳。当前默认配置面向整车
联调，包含：

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

`python3-yaml` 只需安装一次。如果出现 `ModuleNotFoundError: yaml`，说明它没有安装在当前 ROS1
容器实际使用的 Python 环境中。

录制器会检查 `/cloud_registered_body`、`/tf`、`/gps/fix`、`/gnss/gpgga` 和
`/heading` 是否真的存在。缺失时终端每 5 秒告警，但不会中断其他话题；`Ctrl+C` 结束时还会
列出本次从未出现的必录话题。因此不能只看录包程序已经启动，还要确认没有关键话题缺失告警。

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

默认文件前缀为 `vehicle_data`，实际文件名包含启动时间；启用分包后类似
`vehicle_data_20260908_091500_0.bag`。可以通过 `bag_prefix` 按建图、导航、控制器测试等用途临时
命名，不需要修改 launch 文件。

默认使用 LZ4 压缩、每 2048 MB 自动分包，并在磁盘剩余空间低于 8 GB 时安全停止。每次录制还
会生成同名前缀的 JSON 清单。结束时按一次 `Ctrl+C`，看到“录制结束”后再断电或拔移动硬盘。

### 增加相机或其他传感器

编辑：

```text
/home/nuc/lxw/neupan_ws/src/vehicle_bag_recorder/config/vehicle_data.yaml
```

只需在 `topics` 中追加真实话题，不需要修改 Python：

```yaml
topics:
  - /camera/color/image_raw/compressed
  - /camera/color/camera_info
  - /camera/depth/image_raw/compressedDepth
  - /camera/depth/camera_info
```

接入相机后先检查实际话题和带宽：

```bash
rostopic list | grep -E 'camera|image|compressed'
rostopic hz /camera/color/image_raw/compressed
rostopic bw /camera/color/image_raw/compressed
```

优先录制压缩图像以减少磁盘和带宽占用。如果离线算法必须使用无损原图，再录制 `image_raw`；
同时必须录制对应的 `camera_info`、`/tf` 和 `/tf_static`。

## 1.5 Rosbag 离线路缘测试

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

```
wget http://fishros.com/install -O fishros && . fishros
```

ros1,ros1_neupan

[一键安装过程中拉取镜像出现问题 | 鱼香ROS](https://fishros.org.cn/forum/topic/3085/一键安装过程中拉取镜像出现问题/2)

```
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

```
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

```
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

```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j8
sudo make install
```

### Livox ROS Driver 2

```
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

#### 编译 Livox ROS Driver 2

```bash
source /opt/ros/noetic/setup.sh
```

在 Livox ROS Driver 2文件夹下

```
cd src/livox_ros_driver2/
./build.sh ROS1
```

### 配置主板 IP

```bash
#nx super
sudo ifconfig enP8p1s0 192.168.1.100 netmask 255.255.255.0
#nuc
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
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

```
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
```

##### 方法二

```
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

```
python3 -c "import neupan; print('OK')"
```

#### py38

如果你想用python3.8在没有虚拟环境的情况下运行这个项目，可以安装NeuPAN规划器的py38分支：https://github.com/hanruihua/NeuPAN/tree/py38

```
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

```
chmod 777 * src/neupan_ros/src/neupan_core.py
chmod 777 * src/neupan_ros/src/neupan_node.py

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
|                          | d_max                  | 2.0        | 障碍最大感知距离（m）             | float    |
|                          | d_min                  | 0.005      | 障碍最小安全距离（m）             | float    |

| 模块   | 参数名                 | 公式对应关系                                                 | 核心轨迹影响                                                 |
| ------ | ---------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| mpc    | receding               | 预测时域长度 H=10，轨迹长度 = H×step_time                    | 增大→轨迹更长、规划视野更远；减小→反应更快、更灵活           |
|        | step_time              | 控制步长 dt=0.2s，状态转移公式 st+1=Atst+Btut+ct             | 增大→轨迹更新频率降低；减小→更新更频繁、轨迹更精细           |
|        | ref_speed              | 基础代价 C0 中速度跟踪项参考值                               | 增大→轨迹巡航速度更快；减小→更平缓                           |
|        | device                 | 无直接公式关联，仅影响计算效率                               | 固定为 CPU，不影响轨迹形状                                   |
|        | time_print             | 无直接公式关联                                               | 仅控制日志输出，不影响轨迹                                   |
|        | collision_threshold    | 碰撞判定阈值，对应 dist(Z,P)<0.01m 判定碰撞                  | 增大→碰撞判定更宽松；减小→更严格                             |
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
|        | d_max                  | 障碍感知范围：仅处理 dist≤2.0m 的障碍物                      | 增大→感知范围更广、避障更提前；减小→过滤远处障碍、计算量降低 |
|        | d_min                  | 避障代价 Cobs 中最小安全距离                                 | 增大→轨迹离障碍物更远、更保守；减小→更贴障碍物、更激进       |

### 走直线左右摇摆问题

1. **减小max_acce的角加速度（小于0.1）**

   对 Ackermann 车辆，`max_acce` 的第二个值用于限制转向量的变化速度。数值过大时，优化器每次求解都可能快速改变转向角；当横向误差或航向误差存在噪声时，车辆容易出现“向左修正后马上向右修正”的左右摆动。

   适当减小该值可以让转向变化更平滑，减少控制输出的突变。

2. **增大 `receding` 或 `step_time`**

   `receding` 表示预测步数，`step_time` 表示每个预测步的时间，因此预测总时长约为：

   ```
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

```
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

```
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

```
cd /home/hart2204/RoutePlanning/neupan_ws
source devel/setup.bash

# 启动环境 Launch 文件
roslaunch neupan_ros gazebo_limo_env_complex_20.launch
```

#### 终端 2：启动 Neupan 算法节点

启动核心规划算法、Rviz 可视化界面。

```
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

```html
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

```
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

```
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

```
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
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
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

```
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
   sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
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

   ```
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
   
   ```yaml
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

```bash
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
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
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

```
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
roslaunch teb_test outdoor_teb_test.launch
```

“不带任何局部规划”的全局直跟测试，也就是你之前一直在跑的 direct tracker，是：

```
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
roslaunch neupan_ros outdoor_global_only_test.launch direct_tracker_yaw_offset_deg:=0 direct_tracker_projected_goal_stop_tolerance:=1.0
```

如果你想测带局部避障的 `NeuPAN` 全系统，则是：

```
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
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

key：a7ec0a36dedccca86423b86dee3c3a41

私钥：4509ae59e5541e11fa1a3c3f80106c23

```
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

```
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

```
http://t0.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.
0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={level}&TIL
EROW={x}&TILECOL={y}&tk=56127e90e994e759a9dbab1a1f3bfd5a
```

### 下载maprviz

先看你的 ROS 版本：

```
echo $ROS_DISTRO
```

然后安装：

```
sudo apt update
sudo apt install ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTR
```

### 开启g90 maprviz

```
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

卡号：cndh87282
密码：a8888858

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

AT+NTRIP=addr,port,user,password,mount

(其中 addr 为 Ntrip 服务器地址，port 为端口号，user 与 password 是 CORS 账号和

密码，mount 为挂载点。)

如：AT+NTRIP=120.253.239.161,8002,csar10154,a8888168,AUTO

AT+NTRIP=120.253.239.161,8002,cndh87282,a8888858,RTCM33_GRCE

**按一下”shift+Enter”键后再点击发送，**发送成功后会返回【OK】然后保存参数

# 十二、道路视觉与相机

本章汇总道路边界感知。当前实车优先使用 MID360 路缘检测；相机车道线内容作为后续融合方案保留。

## 12.1 MID360 激光雷达路缘检测（当前方案）

激光雷达路缘检测已经存在，并且分为两个包：

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
| `lidar_curb_refinement/TESTING_GUIDE_CN.md` | 低速标定与验收步骤 |

主要话题：

| 话题 | 消息类型 | 作用 |
|---|---|---|
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | FAST-LIO 去畸变后的车体点云输入 |
| `/road_curb/right_edge` | `geometry_msgs/PointStamped` | 检测到的右侧路缘代表点 |
| `/road_curb/lateral_error` | `std_msgs/Float32` | 路径横向修正量 |
| `/road_curb/status` | `std_msgs/String` | JSON 格式的有效性、置信度和拒绝原因 |
| `/osm_global_path_curb_refined` | `nav_msgs/Path` | 供 NeuPAN 或直跟控制器使用的靠右路径 |

启动、标定、靠右闭环和 rosbag 回放命令统一放在本文 [1.3](#13-mid360-右侧路缘检测与靠右行驶)、
[1.4](#14-rosbag-一键录制) 和 [1.5](#15-rosbag-离线路缘测试)，这里不再重复。

路缘 launch 默认启用独立日志，文件名为 `tracker_logs/lidar_curb_时间.jsonl`，`Ctrl+C` 时自动
刷新关闭。它和 `system_run_logger` 的综合导航日志互不覆盖，可以用时间戳对齐分析。

## 12.2 相机道路边界/车道线检测（历史与后续方案）


### 车道线检测：HSV/CLRNet

车道线观测由 `road_vision_refinement` 提供，用于在 OSM 全局路径基础上产生局部视觉横向误差。默认只发布观测结果，不直接修改 NeuPAN 的输入路径。

#### 处理链路与输出

```text
校正 RGB 图像
-> HSV 白色/黄色候选
-> HoughLinesP + 位置/斜率/跨帧筛选
-> 车道线曲线与目标点
-> IPM 投影到 base_link 地面坐标
-> /road_vision/lateral_error
```

自动模式的回退顺序：

```text
center_line -> road_edge -> hold_last_valid -> no_visual_reference
```

主要话题：

| 话题 | 作用 |
|---|---|
| `/road_vision/debug_image` | 紫色为确认的车道线，橙色为目标点 |
| `/road_vision/center_line_mask` | 车道线候选掩码 |
| `/road_vision/road_mask` | 低饱和度道路候选掩码 |
| `/road_vision/status` | JSON 状态、误差、来源和拒绝原因 |
| `/road_vision/lateral_error` | 经 IPM 投影后的米制横向误差 |

`base_link` 坐标约定为 `x` 向前、`y` 向车辆左侧；状态中的正横向误差表示路径应向车辆右侧修正。

#### IPM 标定状态

IPM 矩阵保存在：

```text
src/road_vision_refinement/config/calibration/ipm_points.json
```

已使用两个独立点 `(1.5, 0.3)`、`(1.8, 0.3)` 验证，单点误差均为 `0.225 m`，当前暂按最大误差不超过 `0.25 m` 使用。相机位置、安装高度、俯仰角或图像分辨率变化后，必须重新标定。

### HybridNets：可行驶区域与车道线检测



## 12.3 奥比中光 Pro Plus

ros1：https://github.com/orbbec/ros_astra_camera.git

ros2：[orbbec/OrbbecSDK_ROS2: OrbbecSDK ROS2 wrapper](https://github.com/orbbec/OrbbecSDK_ROS2)

### 依赖

- 基于ros功能包重新依赖 Install libuvc

```bash
cd /home/nuc/lxw/neupan_ws/src/ros_astra_camera-main/libuvc
rm -rf build
mkdir build && cd build
cmake .. && make -j4
```

### 标定

棋盘格生成：[相机校准图案生成器 – calib.io](https://calib.io/zh/pages/camera-calibration-pattern-generator)


# 十三、VLM 与世界模型

本章为后续多模态与世界模型调研，不参与当前室外导航主链路。


## 13.1 Jetson Orin NX Super

### 环境配置与编译

```
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

```
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
