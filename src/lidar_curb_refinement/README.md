# MID360 右侧路沿路径修正

本包把 `lidar_curb_detection` 的“由高度突变提取道路边界”思路接入当前小车系统。
原项目依赖固定扫描线编号，适用于 Velodyne 等机械式雷达；MID360 是非重复扫描，不能直接套用
`nScanRings`。因此本包使用 FAST-LIO 去畸变后的 `/cloud_registered_body`，先转换到
`base_link`，再在车体右前方的高程栅格中检测连续上台阶。

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
