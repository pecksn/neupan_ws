# vehicle_bag_recorder

用于实车离线测试的配置化 ROS1 rosbag 录制工具。Python 只负责读取配置、启动官方
`rosbag record`、监控磁盘并安全结束，因此保留消息原始类型和时间戳。

## 使用

编译并 source 工作区后：

```bash
roslaunch vehicle_bag_recorder lidar_curb_record.launch
```

也可以直接运行 Python：

```bash
rosrun vehicle_bag_recorder bag_recorder.py \
  --config $(rospack find vehicle_bag_recorder)/config/lidar_curb.yaml
```

按 `Ctrl+C` 后程序会向 rosbag 发送 `SIGINT`，等待它完成索引写入。每次输出包括带日期时间的
bag（达到配置大小后自动分包）以及同名前缀的 JSON 清单。默认保存到工作区的 `rosbags/`。

临时改变保存目录或名称：

```bash
roslaunch vehicle_bag_recorder lidar_curb_record.launch \
  output_directory:=/media/usb/rosbags bag_prefix:=bridge_test
```

## 添加相机或其他传感器

编辑 [config/lidar_curb.yaml](config/lidar_curb.yaml) 的 `topics`，增加相机实际话题即可，Python
不需要修改。建议先用以下命令确认名称和带宽：

```bash
rostopic list | grep camera
rostopic bw /camera/color/image_raw
```

相机优先录制 `image_raw/compressed`；如果离线算法必须使用无损原图，再录 `image_raw`。务必同时
录制对应的 `camera_info`、`/tf` 和 `/tf_static`。

回放示例：

```bash
rosbag info /home/nuc/lxw/neupan_ws/rosbags/lidar_curb_test_*.bag
rosbag play --clock /home/nuc/lxw/neupan_ws/rosbags/lidar_curb_test_*.bag
```

使用仿真时间的离线节点应设置：

```bash
rosparam set use_sim_time true
```
