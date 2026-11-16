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
bag（达到配置大小后自动分包）以及同名前缀的 JSON 清单。每次录制默认按北京时间保存到
独立的 `rosbags/rosbagYYYYMMDD_HHMMSS/` 目录。

默认必录检查包含 `/cloud_registered_body`、`/tf`、GPS Fix、GGA、航向、
`/camera/image_raw` 和 `/camera/camera_info`。关键话题没有发布者时
终端每 5 秒告警，但其他话题仍继续录制；有发布者不代表已有消息写入bag。

正常结束时读取本次各分包索引，不解码图像/点云，将实际话题类型与消息数写入原 JSON 清单的 `recording_result`：

- `topic_message_counts`：所有分包按话题累计的消息数。
- `required_topics_without_messages`：未查到消息的必录话题。
- `shutdown`、`return_code`、`process_running`：关闭请求原因、阶段异常、子进程退出码及是否仍存活；状态无法查询时为null，不能视为已退出。
- `index_verification_complete`：记录子进程已退出、所有bag索引核验成功且没有 `.bag.active` 遗留。
- `all_required_topics_recorded`：索引核验完整且所有必录话题有消息；不代表数据质量或整段无断流。

无文件、损坏索引或未完成分包会明确标记未核验完成，不自动修复或删除原bag。强制杀进程/断电时可能无法写入结束结果，此时清单缺少 `recording_result`，不能视为核验通过。

SIGINT等待20秒、SIGTERM再等待5秒后仍未退出，也会尽力保存不完整清单，不新增SIGKILL。发送信号时子进程恰好退出等异常会记录在 `shutdown.errors`。清单更新失败保留原文件，重复调用stop只重试未成功的清单，不重复发信号。

三份配置还保留原始模式请求、遥控输入、GNSS解状态、目标/到达事件、经纬度路径和 `/vehicle/command_audit`（驱动串口发送审计，不是实测轮速）。模式请求及目标等事件可能整趟不产生，未被加入连续传感器必录告警；检查 `topic_message_counts` 时结合本次实际操作。

雷达路缘检测已移除，录包配置不再录制相关专用话题；原始雷达、FAST-LIO点云与避障话题保留。

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
rosbag info /home/nuc/lxw/neupan_ws/rosbags/rosbag*/vehicle_data_*.bag
rosbag play --clock /home/nuc/lxw/neupan_ws/rosbags/rosbag*/vehicle_data_*.bag
```

回放后可先确认 RGB 帧已正常恢复：

```bash
rosrun image_view image_view image:=/camera/image_raw
```

使用仿真时间的离线节点应设置：

```bash
rosparam set use_sim_time true
```
