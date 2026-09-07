# MID360 右侧路沿检测测试指南

本模块只使用 MID360 点云，不包含相机、HSV、CLRNet 或 IPM。

## 启动

先启动 FAST-LIO，确认 `/cloud_registered_body` 可用且能通过 TF 转换到 `base_link`：

```bash
rostopic echo -n 1 /cloud_registered_body/header
rosrun tf tf_echo base_link body
```

只观察检测结果：

```bash
roslaunch lidar_curb_refinement road_curb_debug.launch
rostopic echo /road_curb/status
```

`valid=true` 时，状态中的 `right_distance_m` 是车辆原点到右侧路沿的距离；
`lateral_error_m` 是建议给全局路径的横移量，正值向右、负值向左。

默认目标右侧净距为 `0.55 m`。例如实际路沿在右侧 `0.90 m`，输出约为
`+0.35 m`，使路径逐步向右靠近；实际只有 `0.35 m` 时，输出约为 `-0.20 m`，使路径向左远离路沿。

## 接入路径测试

先在 RViz 对比原路径和修正路径，未确认方向前不要让 NeuPAN 使用输出路径：

```bash
roslaunch lidar_curb_refinement road_curb_debug.launch \
  start_path_refine:=true \
  input_path_topic:=/osm_global_path \
  output_path_topic:=/osm_global_path_curb_refined
```

满足以下条件后才允许 NeuPAN 订阅 `/osm_global_path_curb_refined`：

- 路沿连续出现时 `valid=true`，丢失时变为 `false`；
- 室外直线路段中，右侧净距变化方向正确；
- 道路边停放车辆、草坪和排水沟不会造成持续有效的假路沿；
- 低速测试中路径横移平滑，且不超过 `max_correction_m=0.45`。

## 常用调参

- `desired_clearance_m`：期望与右侧路沿保持的距离。
- `min_step_height`：最小路沿高度，默认 `0.045 m`；漏检时可逐步降到 `0.03 m`。
- `min_x_bins`：要求路沿沿前方连续的栅格数量；误检时增大，漏检时小幅减小。
- `confirm_frames`：连续确认帧数；误跳时增大，响应过慢时减小。

参考项目源码位于 `src/lidar_curb_detection`。它依赖固定激光扫描线，不能直接处理 MID360 的
非重复扫描模式；本系统实际运行的是本包的高程栅格版本。

## 整套系统启用

观察模式（不改变控制路径）：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true enable_lidar_curb:=true enable_lidar_curb_control:=false
```

上述检查全部通过后，才启用闭环：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true enable_lidar_curb:=true enable_lidar_curb_control:=true
```

修正采用路径行进方向的右法向量，而不是地图固定方向，所以路径反向时右侧也会随行驶方向反转。
