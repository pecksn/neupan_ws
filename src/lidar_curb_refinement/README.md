# MID360 右侧路沿路径修正

本包把 `lidar_curb_detection` 的“由高度突变提取道路边界”思路接入当前小车系统。
原项目依赖固定扫描线编号，适用于 Velodyne 等机械式雷达；MID360 是非重复扫描，不能直接套用
`nScanRings`。因此本包使用 FAST-LIO 去畸变后的 `/cloud_registered_body`，先转换到
`base_link`，再在车体右前方的高程栅格中检测连续上台阶。

## Python/ROS 依赖

NeuPAN 可以继续使用 `conda activate lxw`。路缘 launch 会通过 `launch-prefix` 强制三个路缘
节点使用 `/usr/bin/python3`，避免 `tf2_sensor_msgs` 在 Conda 中找不到系统安装的 `PyKDL`。
在 ROS1 Noetic 容器中安装并验证：

```bash
sudo apt update
sudo apt install python3-pykdl ros-noetic-tf2-sensor-msgs
/usr/bin/python3 -c "import PyKDL, rospy, tf2_sensor_msgs; print('curb ROS Python OK')"
```

修改脚本后需要重新 `catkin_make` 并重新 source 工作区。

## 数据流

```text
/cloud_registered_body
        -> lidar_curb_node.py
        -> /road_curb/lateral_error
        -> path_refine_node.py + /osm_global_path
        -> /osm_global_path_curb_refined
        -> NeuPAN 或 global_path_tracker
```

`base_link` 采用 ROS 约定：x 向前、y 向左，所以右侧路沿的 y 为负。路径修正使用路径行进方向
的右法向量 `(tangent_y, -tangent_x)`，因此同一条道路正向、反向行驶时都会偏向各自行驶方向的
右侧，而不是固定向地图的某个方向偏移。

启动本包时默认同时运行 `curb_run_logger.py`，以 10 Hz 将状态、横向误差、右侧边缘点及路径
概况持续写入 `/home/nuc/lxw/neupan_ws/tracker_logs/lidar_curb_时间.jsonl`。按 `Ctrl+C` 后自动
刷新并关闭文件。临时关闭日志可传入 `enable_log:=false`。

```bash
roslaunch lidar_curb_refinement road_curb_debug.launch
tail -f "$(ls -1t /home/nuc/lxw/neupan_ws/tracker_logs/lidar_curb_*.jsonl | head -1)"
```

正常测试不需要运行 `rostopic echo`。

## 系统接入

仅检测、暂不影响控制：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true enable_lidar_curb:=true enable_lidar_curb_control:=false
```

完成低速标定后，使 NeuPAN 使用修正路径：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true enable_lidar_curb:=true enable_lidar_curb_control:=true
```

全局路径直跟测试同样支持这两个参数：

```bash
roslaunch neupan_ros outdoor_global_only_test.launch \
  enable_lidar_curb:=true enable_lidar_curb_control:=true
```

必须同时开启两个开关。若只开 `enable_lidar_curb_control`，修正路径没有发布者，控制器不会收到
路径；这是为了防止检测器未运行时静默切回未经确认的控制源。

实车启用控制前请按 [TESTING_GUIDE_CN.md](TESTING_GUIDE_CN.md) 完成方向、净距、误检和丢失测试。
