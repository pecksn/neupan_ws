# vehicle_bag_recorder

用于实车离线测试的配置化 ROS1 rosbag 录制工具。Python 只负责读取配置、启动官方
`rosbag record`、监控磁盘并安全结束，因此保留消息原始类型和时间戳。

## 使用

编译并 source 工作区后：

```bash
sudo apt install python3-yaml
python3 -c "import yaml; print('PyYAML OK')"
cd /home/nuc/lxw/neupan_ws
catkin_make
source devel/setup.bash
roslaunch vehicle_bag_recorder record.launch
```

也可以直接运行 Python：

```bash
rosrun vehicle_bag_recorder bag_recorder.py \
  --config $(rospack find vehicle_bag_recorder)/config/vehicle_data.yaml
```

按 `Ctrl+C` 后程序会向 rosbag 发送 `SIGINT`，等待它完成索引写入。每次输出包括带日期时间的
bag（达到配置大小后自动分包）以及同名前缀的 JSON 清单。默认保存到工作区的 `rosbags/`。

默认必录检查包含 `/cloud_registered_body`、`/tf`、GPS Fix、GGA、航向、
`/camera/image_raw` 和 `/camera/camera_info`。关键话题没有发布者时
终端每 5 秒告警，但其他话题仍继续录制；结束时会再次列出整次录制从未出现的话题。

默认话题列表还会保存 `/road_curb/roi_cloud` 和 `/road_curb/candidate_cloud`，用于在 RViz
中把路缘检测区域、候选高度突变点与原始点云叠加分析。

临时改变保存目录或名称：

```bash
roslaunch vehicle_bag_recorder record.launch \
  output_directory:=/media/usb/rosbags bag_prefix:=bridge_test
```

## 相机图像与 HybridNets

默认配置会把 USB Camera3 的 `/camera/image_raw` (`sensor_msgs/Image`) 和
`/camera/camera_info` 直接录入 bag，不会额外导出 JPG/PNG。`/camera/image_raw` 是相机节点
经 180° 旋转后的彩色帧，回放时可由 ROS 适配节点通过 `cv_bridge` 转为 OpenCV
图像，再送入 HybridNets。HybridNets 本身不直接读 rosbag，因此推理端仍需订阅这个话题。

录制前可确认话题类型、编码和带宽：

```bash
rostopic type /camera/image_raw
rostopic echo -n 1 /camera/image_raw/encoding
rostopic bw /camera/image_raw
```

默认不同时录制 `/camera/image_raw/compressed`，避免同一帧图像重复占用磁盘。Bag 的
LZ4 压缩仍会对原始 `Image` 消息生效。如果以后更换相机，修改
[config/vehicle_data.yaml](config/vehicle_data.yaml) 中 `topics` 和 `required_topics` 的话题名即可。

回放示例：

```bash
rosbag info /home/nuc/lxw/neupan_ws/rosbags/vehicle_data_*.bag
rosbag play --clock /home/nuc/lxw/neupan_ws/rosbags/vehicle_data_*.bag
```

回放后可先确认 RGB 帧已正常恢复：

```bash
rosrun image_view image_view image:=/camera/image_raw
```

使用仿真时间的离线节点应设置：

```bash
rosparam set use_sim_time true
```
