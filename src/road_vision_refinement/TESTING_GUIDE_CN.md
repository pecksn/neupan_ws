# 中心线与道路边缘联合测试指南

本文档用于验证 `road_vision_refinement`：优先根据中心线生成右侧目标，中心线不可用时尝试道路右边缘；两者都不可用时短暂保持上一结果，再平滑回到原始路径。

测试期间车辆保持静止，不要接入 NeuPAN。

## 1. 当前算法

### 中心线检测

```text
HSV 白色/黄色阈值
-> 形态学去噪
-> HoughLinesP 线段检测
-> 长度、倾角、历史位置筛选
-> 一次/二次多项式拟合
-> 在中心线右侧生成目标
```

`center_line_mask` 显示颜色候选，调试图中的紫色曲线是已确认的中心线，橙色点是目标。默认还要求道路候选区域有效（`center_line_requires_road=true`），再通过落点范围、道路行驶方向的斜率约束、连续 `line_confirm_frames` 帧确认、跨帧位置/方向跳变拒绝和曲线指数平滑，未确认的线不会生成紫色曲线。

### 道路边缘检测

```text
HSV 低饱和度道路候选
-> 形态学去噪
-> 最大连通轮廓
-> 水平扫描估计轮廓范围
-> 将轮廓右侧作为道路边缘候选
```

当前 `road_edge` 不是严格的路沿检测，可能受墙体、车辆和阴影干扰。现在还要求候选轮廓覆盖多个水平扫描线并接触 ROI 底部，但这仍是几何启发式，必须结合 `road_mask` 人工确认。

### 自动模式

`target_mode=auto` 的优先级：

```text
center_line -> road_edge -> hold_last_valid -> no_visual_reference
```

两种视觉参考都失效时，默认保持上一结果 `1 s`，之后按 `invalid_decay_alpha=0.08` 平滑回到原始路径。

## 2. 准备

```
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
sudo ifconfig enp114s0 192.168.1.100 netmask 255.255.255.0
roslaunch neupan_ros full_system_run.launch outdoor_mode:=true
```

室外主 launch 已配置 `base_link -> camera_link`：平移 `(0.18, 0, 0.30)m`，姿态为 `roll=0deg, pitch=+8deg（向下）, yaw=0deg`。相机启动后检查：

```bash
rosrun tf tf_echo base_link camera_link
rosrun tf tf_echo base_link camera_color_optical_frame
```

如果 URDF 或其他节点已经发布相同 TF，启动室外 launch 时增加 `publish_base_camera_tf:=false`，禁止两个节点同时发布相同父子变换。

### 获取像素坐标并计算 IPM

IPM 使用成对数据：同一个地面点的图像像素 `(u,v)`，以及它在 `base_link` 地面坐标中的 `(x,y)`。约定 `x` 向车辆前方、`y` 向车辆左侧，单位为米；所有点必须处于同一水平地面。

先启动校正图：

```bash
ROS_NAMESPACE=/camera/rgb rosrun image_proc image_proc
rostopic hz /camera/rgb/image_rect_color
```

在地面贴至少 4 个、推荐 8 个标记，并用卷尺记录相对 `base_link` 的坐标。例如：

```text
(1.0, -0.5) (1.0, 0.5)
(2.0, -1.0) (2.0, 1.0)
(3.0, -1.5) (3.0, 1.5)
(4.0, -2.0) (4.0, 2.0)
```

保存一张校正图：

```bash
rosrun image_view image_saver \
  image:=/camera/rgb/image_rect_color \
  _filename_format:=/home/nuc/lxw/neupan_ws/src/road_vision_refinement/config/calibration/images/ipm_ground_%04i.png
rosservice call /image_saver/save
```

### 当前实际使用的 4 点标定

你当前实际使用的是 4 个地面点：

```text
(2.0, -0.5)
(2.0,  0.5)
(3.0, -0.5)
(3.0,  0.5)
```

用点击工具获取像素坐标并计算 IPM。点击顺序必须与 `--ground` 完全一致：

```bash
python3 src/road_vision_refinement/scripts/pick_ipm_points.py \
  /home/nuc/lxw/neupan_ws/src/road_vision_refinement/config/calibration/images/ipm_ground_0000.png \
  --ground '[[2.0,-0.5],[2.0,0.5],[3.0,-0.5],[3.0,0.5]]' \
  --output /home/nuc/lxw/neupan_ws/src/road_vision_refinement/config/calibration/ipm_points.json
```

程序会打开图片。按 `--ground` 的顺序依次点击地面标记中心，直接得到像素 `(u,v)`，并将矩阵保存到 `config/calibration/ipm_points.json`。

### 当前实际使用的新点验证

你当前使用验证图片：

```text
ipm_verify_0005.png
```

验证点为：

```text
(2.5, 0.2)
(3.0, 0.2)
```

验证已有矩阵时，按顺序点击这两个新点，执行：

```bash
python3 src/road_vision_refinement/scripts/verify_ipm_points.py src/road_vision_refinement/config/calibration/images/ipm_verify_0005.png --ground '[[2.5,0.2],[3.0,0.2]]' --matrix src/road_vision_refinement/config/calibration/ipm_points.json
```

程序输出每个点的预测坐标和误差。由于两个验证点位于当前标定范围 `x=2~3m` 内，建议误差小于 `0.05~0.10m`；若明显超过 `0.10m`，先检查点击位置、地面坐标和相机是否在两次拍摄之间移动。

## 3. 启动联合检测

终端 1：

```bash
roscore
```

终端 2：

```bash
roslaunch astra_camera astra_pro_plus.launch depth_align:=true enable_depth:=true
```

确认相机：

```bash
rostopic hz /camera/rgb/image_raw
rqt_image_view /camera/rgb/image_raw
```

终端 3，只启动检测，不修正路径：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  image_topic:=/camera/rgb/image_raw \
  target_mode:=auto \
  enable_path_refine:=false
```

分别查看以下话题：

```bash
rqt_image_view /road_vision/debug_image
rqt_image_view /road_vision/center_line_mask
rqt_image_view /road_vision/road_mask
rostopic echo /road_vision/status
```

同一帧会同时生成中心线和道路 mask，最终使用的来源由 `/road_vision/status` 中的 `source` 表示。

状态中还会显示 `line_candidate_confidence`、`line_candidate_confirmed`、`road_candidate_confidence` 和 `road_bottom_touch_ratio`，可用于区分“没检测到”和“检测到但未通过确认”。

## 4. 场景验收

| 场景 | 图像要求 | 期望状态 |
|---|---|---|
| 中心线清晰 | 紫色曲线贴着中心线，橙色点在其右侧 | `source=center_line`, `valid=true` |
| 无中心线、右边缘清晰 | `road_mask` 覆盖路面，橙色点在边缘内侧 | `source=road_edge`, `valid=true` |
| 两者短暂丢失 | 目标和误差保持上一结果 | `reason=hold_last_valid` |
| 两者持续丢失 | 不再输出有效视觉参考 | `valid=false`, `reason=no_visual_reference` |

出现以下情况应停止，不进入路径测试：

- 紫色曲线落在墙、地砖缝、桌腿或反光区域；
- `road_mask` 覆盖墙体或几乎整幅图；
- 道路候选右边缘就是图像最右侧；
- 橙色目标点落到道路外；
- `source` 在静止场景下频繁跳变。

## 5. 录包与离线复测

分别经过“有中心线、只有右边缘、两者都没有”的场景：

```bash
mkdir -p /home/nuc/lxw/neupan_ws/bags/road_test
rosbag record -O /home/nuc/lxw/neupan_ws/bags/road_test/vision_modes.bag \
  /camera/rgb/image_raw \
  /camera/rgb/camera_info \
  /road_vision/debug_image \
  /road_vision/center_line_mask \
  /road_vision/road_mask \
  /road_vision/status \
  /road_vision/lateral_error \
  /osm_global_path \
  /osm_global_path_refined \
  /scan /tf /tf_static
```

停止录制后检查：

```bash
rosbag info /home/nuc/lxw/neupan_ws/bags/road_test/vision_modes.bag
```

离线播放：

```bash
rosparam set use_sim_time true
rosbag play --clock /home/nuc/lxw/neupan_ws/bags/road_test/vision_modes.bag
```

## 6. 路径平滑测试

只有第 4 节全部通过后才执行，并保持车辆静止：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  image_topic:=/camera/rgb/image_raw \
  target_mode:=auto \
  enable_path_refine:=true \
  invalid_hold_sec:=1.0 \
  invalid_decay_alpha:=0.08
```

在 RViz 同时显示：

```text
/osm_global_path
/osm_global_path_refined
```

依次测试：

```text
中心线可见 -> 只有道路边缘 -> 两者遮挡 -> 恢复中心线
```

应满足：

- 来源切换时路径连续；
- 两者失效 1 秒内保持原偏移；
- 持续失效时平滑回到原始路径；
- 恢复中心线时平滑重新产生偏移；
- `/osm_global_path` 始终不被修改。

## 7. 室外导航接入状态

目前**没有自动接入室外 NeuPAN**：

- `road_vision_debug.launch` 默认 `enable_path_refine=false`；
- `full_system_run.launch` 没有启动视觉包；
- `outdoor_initial_path_topic` 默认仍是 `/osm_global_path`；
- `/osm_global_path_refined` 目前只用于调试。

在完成检测验收、相机标定和米制坐标转换前，不要让 NeuPAN 订阅 `/osm_global_path_refined`。

## 8. 当前问题与后续顺序

| 当前问题 | 后续解决方式 |
|---|---|
| `lateral_error_m` 是像素比例，不是真实米制误差 | 标定相机内参和 `base_link -> camera` 外参，增加 IPM 地面投影 |
| `road_edge` 只是颜色轮廓右边缘 | 加入边缘连续性、地面几何和 Livox 路沿校验 |
| 固定 HSV/Hough 容易受光照影响 | 用录包覆盖晴天、阴影、磨损标线并调阈值和置信度 |
| 图像右侧被固定认为车辆右侧 | 根据局部路径方向计算右法向 |
| 整条全局路径统一平移 | 只修正车辆前方 `5~15 m` 的局部路径并随距离衰减 |
| 摄像头和 Livox 尚未融合 | 两者先统一到 `base_link` 米制坐标，再做障碍/边缘安全校验 |

建议顺序：

```text
完成本文检测测试
-> 相机内外参和 Livox 外参
-> 视觉结果转换为 base_link 米制坐标
-> 局部路径修正
-> Livox 安全约束
-> 低速接入室外 NeuPAN
```
