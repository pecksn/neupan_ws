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

## 2. 标定与验证

### 2.1 单张图片保存工具

`save_one_image.py` 会等待下一帧、保存一张图片，然后自动退出，不需要 `image_saver` 或 `rosservice call`：

```bash
rosrun road_vision_refinement save_one_image.py IMAGE_TOPIC OUTPUT_FILE
```

以下命令都要求相机话题已经启动；如果尚未启动，先执行第 3 节的一体化 launch。IPM 必须使用校正图 `/camera/rgb/image_rect_color`，不能使用原始图。

### 2.2 相机内参标定（需要时执行）

如果 `/camera/rgb/camera_info` 已有正确内参，并且 `/camera/rgb/image_rect_color` 能正常输出，可跳过本节。

真正标定内参时，使用棋盘格的实际内角点数量和方格边长。例如棋盘格为 `8x6` 个内角点、方格边长 `0.108 m`：

```bash
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 --square 0.108 \
  image:=/camera/rgb/image_raw camera:=/camera/rgb
```

从不同距离和角度采集棋盘格，窗口中的 `CALIBRATE` 可用后依次点击 `CALIBRATE`、`SAVE`。生成的 YAML 才是内参结果。

如需额外保存一张棋盘格原始图留档：

```bash
rosrun road_vision_refinement save_one_image.py \
  /camera/rgb/image_raw \
  /home/nuc/lxw/neupan_ws/src/road_vision_refinement/config/calibration/images/camera_calibration.png
```

单张棋盘格图片只用于留档，不能替代上面的多姿态内参标定。

### 2.3 IPM 标定

约定 `base_link` 中 `x` 向车辆前方、`y` 向车辆左侧，单位为米。保持车辆和相机不动，在同一水平地面放置以下 4 个标记：

```text
(2.0, -0.5)
(2.0,  0.5)
(3.0, -0.5)
(3.0,  0.5)
```

保存一张 IPM 校正图：

```bash
rosrun road_vision_refinement save_one_image.py \
  /camera/rgb/image_rect_color \
  /home/nuc/lxw/neupan_ws/src/road_vision_refinement/config/calibration/images/ipm_ground.png
```

计算 IPM 矩阵：

```bash
python3 src/road_vision_refinement/scripts/pick_ipm_points.py \
  src/road_vision_refinement/config/calibration/images/ipm_ground.png \
  --ground '[[2.0,-0.5],[2.0,0.5],[3.0,-0.5],[3.0,0.5]]' \
  --output src/road_vision_refinement/config/calibration/ipm_points.json
```

程序打开图片后，严格按照 `--ground` 中的顺序点击 4 个标记中心。结果写入 `config/calibration/ipm_points.json`。

### 2.4 独立新点验证

不要使用刚才参与标定的 4 个点验证。保持车辆和相机不动，把标记移到新的已知位置。当前实际验证点为：

```text
(1.5, 0.3)
(1.8, 0.3)
```

保存新的校正图：

```bash
rosrun road_vision_refinement save_one_image.py \
  /camera/rgb/image_rect_color \
  /home/nuc/lxw/neupan_ws/src/road_vision_refinement/config/calibration/images/ipm_verify.png
```

验证矩阵：

```bash
python3 src/road_vision_refinement/scripts/verify_ipm_points.py \
  src/road_vision_refinement/config/calibration/images/ipm_verify.png \
  --ground '[[1.5,0.3],[1.8,0.3]]' \
  --matrix src/road_vision_refinement/config/calibration/ipm_points.json
```

按 `--ground` 顺序点击两个新标记。程序会输出预测坐标、单点误差、最大误差和平均误差。

### 2.5 当前状态

当前 4 点 IPM 标定和两个独立新点验证已经完成，两个验证点误差均为 `0.225 m`，暂按 `max_error <= 0.25 m` 验收。矩阵保存在 `config/calibration/ipm_points.json`，无需重复执行第 2.3、2.4 节。只有相机位置或俯仰角发生变化时才重新标定。

现在继续第 3 节。

## 3. 下一步：静止联合检测

车辆保持静止，不要启动 `full_system_run.launch`，也不要接入 NeuPAN。`road_vision_debug.launch` 现在会自动启动 Astra、`image_proc` 和视觉节点，并使用 `/camera/rgb/image_rect_color`。

RGB 内参默认从 `config/calibration/astra_rgb_camera.yaml` 加载，该文件来自标定时保存的 640×480 `CameraInfo`。

启动前在当前终端执行：

```bash
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

只启动检测，不修正路径：

```bash
roslaunch road_vision_refinement road_vision_debug.launch
```

`roslaunch` 会在需要时自动启动 ROS Master，不必单独运行 `roscore`。如果相机或 `image_proc` 已由其他 launch 启动，使用对应开关避免重复节点：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_camera:=false \
  start_image_proc:=false
```

视觉节点默认每秒在当前终端输出一次状态，不需要另外执行 `rostopic echo`。需要同时打开三个调试图窗口时执行：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_debug_viewers:=true
```

当前 Conda 环境中的 `rostopic` 缺少 `Cryptodome`，`rqt_image_view` 缺少 PyQt5，因此一体化 launch 使用视觉节点自身打印状态，并使用 C++ `image_view` 显示图片，不依赖这两个 Python 工具。

同一帧会同时生成中心线和道路 mask，最终使用的来源由 `/road_vision/status` 中的 `source` 表示。

状态中还会显示 `line_candidate_confidence`、`line_candidate_confirmed`、`road_candidate_confidence` 和 `road_bottom_touch_ratio`，可用于区分“没检测到”和“检测到但未通过确认”。

状态中 `metric_calibrated=true` 表示 IPM 已加载。`target_ground_x/y` 是橙色目标点在 `base_link` 地面坐标中的位置；`lateral_error_m` 为真实米制横向误差，正值表示路径向车辆右侧修正。当前仍保持 `enable_path_refine=false`，本节只验收检测和米制输出。

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
  /camera/rgb/image_rect_color \
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

只有第 4 节全部通过，并确认状态中 `metric_calibrated=true`、米制误差方向正确后才执行。测试时车辆仍须保持静止：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
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

在完成检测和米制误差验收前，不要让 NeuPAN 订阅 `/osm_global_path_refined`。

## 8. 当前问题与后续顺序

| 当前问题 | 后续解决方式 |
|---|---|
| IPM 验证误差约 `0.225 m` | 暂按 `0.25 m` 验收，实车低速测试时继续记录并复核系统偏差 |
| `road_edge` 只是颜色轮廓右边缘 | 加入边缘连续性、地面几何和 Livox 路沿校验 |
| 固定 HSV/Hough 容易受光照影响 | 用录包覆盖晴天、阴影、磨损标线并调阈值和置信度 |
| 图像右侧被固定认为车辆右侧 | 根据局部路径方向计算右法向 |
| 整条全局路径统一平移 | 只修正车辆前方 `5~15 m` 的局部路径并随距离衰减 |
| 摄像头和 Livox 尚未融合 | 两者先统一到 `base_link` 米制坐标，再做障碍/边缘安全校验 |

建议顺序：

```text
IPM 标定、验证和视觉节点接入（已完成）
-> 静止联合检测和米制误差验收（当前下一步）
-> 相机外参和 Livox 外参复核
-> 局部路径修正
-> Livox 安全约束
-> 低速接入室外 NeuPAN
```
