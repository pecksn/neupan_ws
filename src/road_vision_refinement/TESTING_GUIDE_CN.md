# road_vision_refinement 测试与使用指南

本文档用于验证相机车道线检测、IPM 米制投影和可选的 NeuPAN 路径修正。
默认只观察视觉结果，不修改 NeuPAN 的原始全局路径。没有完成室外低速验收前，
不要让 NeuPAN 订阅 `/osm_global_path_refined`。

## 1. 当前处理链路

### 1.1 HSV/Hough（默认）

```text
BGR 图像
 -> HSV 白色/黄色阈值
 -> 形态学去噪
 -> HoughLinesP 线段检测
 -> 长度、斜率、位置和跨帧跳变筛选
 -> 一次/二次曲线拟合
 -> 连续帧确认
 -> IPM 投影到 base_link 地面坐标
```

HSV 是颜色空间，OpenCV 是实现 BGR 转 HSV、阈值分割、形态学和 Hough 检测的
计算机视觉库。HSV/OpenCV 不会判断车辆位于车道线哪一侧；`center_line_side_mode`
负责根据线在图像中心的左/右位置选择目标侧。

`target_mode=auto` 的视觉优先级为：

```text
center_line -> road_edge -> hold_last_valid -> no_visual_reference
```

室内地面通常也满足“低饱和度道路”阈值，`road_candidate_confidence=1.0` 不代表
真实道路。墙边、箱体、桌腿和地面接缝也可能通过颜色或几何筛选，室内场景只能
用于验证节点状态和拒绝逻辑，不能用于验证道路识别精度。

### 1.2 CLRNet（可选）

工作区已接入独立的 `clrnet_lane_node.py`。它发布 `/road_vision/clrnet_mask`，
主节点继续负责 Hough 几何筛选、左右侧目标、连续帧确认、IPM 和误差输出。

```text
CLRNet lane mask -> road_vision_node 几何筛选 -> IPM -> lateral_error
```

支持三种模式：

| `line_detector` | 行为 |
|---|---|
| `hsv` | 只使用 HSV/Hough，默认值 |
| `clrnet` | 优先使用 CLRNet；模型未就绪或掩码为空时回退 HSV |
| `hybrid` | 合并 CLRNet 和 HSV 掩码后再筛选 |

当前默认配置是 CULane DLA-34，权重位置为：

```text
src/CLRNet/weights/culane_dla34.pth
```

该通用权重不保证适合校园道路。最终应使用本机室外图像标注后微调。

## 2. 环境与一体化启动

### 2.1 Python 环境

不需要另建独立的 `clrnet` Conda 环境。启动 CLRNet 节点的 Python 必须同时能
导入 `rospy`、`cv_bridge`、`torch`、`mmcv` 和 CLRNet 依赖。

```bash
conda activate lxw
python -c "import rospy, cv_bridge, torch, mmcv; print(torch.__version__)"
```

NeuPAN 需要 `torch>=2.1.0`。不要直接安装上游 CLRNet 固定的 `torch==1.8.0`，
否则会降级 NeuPAN。CPU 环境补充依赖：

```bash
python -m pip install \
  mmcv==1.2.5 addict yapf pandas scikit-learn \
  scipy tqdm imgaug Shapely==1.7.0 ujson \
  timm pathspec ptflops pytorch_warmup
```

如果误装了 `torch==1.8.0`，恢复 NeuPAN 的 CPU 版本：

```bash
conda activate lxw
python -m pip install --force-reinstall \
  torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cpu
```

### 2.2 编译工作区

每次修改 Python、launch 或安装新脚本后执行：

```bash
cd /home/nuc/lxw/neupan_ws
catkin_make --pkg road_vision_refinement
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 2.3 启动视觉节点

默认启动 Astra、`image_proc` 和视觉节点，不启动路径修正：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  enable_path_refine:=false
```

启用 CLRNet 和三个调试窗口：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_clrnet:=true \
  line_detector:=clrnet \
  clrnet_device:=cpu \
  start_debug_viewers:=true \
  enable_path_refine:=false
```

如果相机或 `image_proc` 已由其他 launch 启动：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_camera:=false start_image_proc:=false \
  start_debug_viewers:=true
```

`start_debug_viewers` 默认是 `false`，没有图像窗口时检查是否显式设置为 `true`。

## 3. 图像保存、相机标定和 IPM

### 3.1 保存一张图

相机话题启动后，以下命令等待一帧、保存一张图片并自动退出，不需要
`image_saver` 或 `rosservice call`：

```bash
rosrun road_vision_refinement save_one_image.py \
  /camera/rgb/image_rect_color OUTPUT_FILE.png
```

IPM 必须使用 `/camera/rgb/image_rect_color`，不能使用原始图。

### 3.2 相机内参（需要时）

如果 `/camera/rgb/camera_info` 正确且校正图正常，可跳过。重新标定示例：

```bash
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 --square 0.108 \
  image:=/camera/rgb/image_raw camera:=/camera/rgb
```

### 3.3 IPM 标定

坐标约定：`base_link.x` 向前，`base_link.y` 向车辆左侧，单位为米。将 4 个标记
放在同一水平地面：

```text
(2.0, -0.5)  (2.0, 0.5)
(3.0, -0.5)  (3.0, 0.5)
```

保存图片并计算矩阵：

```bash
rosrun road_vision_refinement save_one_image.py \
  /camera/rgb/image_rect_color \
  src/road_vision_refinement/config/calibration/images/ipm_ground.png

python3 src/road_vision_refinement/scripts/pick_ipm_points.py \
  src/road_vision_refinement/config/calibration/images/ipm_ground.png \
  --ground '[[2.0,-0.5],[2.0,0.5],[3.0,-0.5],[3.0,0.5]]' \
  --output src/road_vision_refinement/config/calibration/ipm_points.json
```

点击顺序必须与 `--ground` 完全一致。矩阵保存在
`config/calibration/ipm_points.json`。

### 3.4 独立点验证

不要使用参与标定的 4 个点。当前独立验证点为：

```text
(1.5, 0.3)
(1.8, 0.3)
```

```bash
rosrun road_vision_refinement save_one_image.py \
  /camera/rgb/image_rect_color \
  src/road_vision_refinement/config/calibration/images/ipm_verify.png

python3 src/road_vision_refinement/scripts/verify_ipm_points.py \
  src/road_vision_refinement/config/calibration/images/ipm_verify.png \
  --ground '[[1.5,0.3],[1.8,0.3]]' \
  --matrix src/road_vision_refinement/config/calibration/ipm_points.json
```

已完成的验证结果：两个点误差均为 `0.225 m`，当前暂按 `max_error<=0.25 m` 验收。
相机位置、安装高度或俯仰角变化后必须重新标定和验证。

## 4. 室内静止测试

室内只验证程序是否安全拒绝无道路参考，不验证车道线准确率。车辆保持静止，
不开启路径修正：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_clrnet:=true line_detector:=clrnet \
  start_debug_viewers:=true enable_path_refine:=false
```

无真实车道线时，期望最终状态为：

```text
valid=false
source=none
reason=no_visual_reference
```

如果室内出现 `source=center_line`、`reason=ok`，通常是 CLRNet 将箱体/门框/地面
接缝作为候选线，或 HSV 道路掩码把地面判成道路。重点检查：

```text
line_hough_count
line_reject_reason
line_candidate_confidence
road_candidate_confidence
center_line_x
```

近垂直设备边缘现在会被斜率门限拒绝；仍然误检时不要把室内结果用于调高置信度，
应直接停止路径联调并保存样本，后续增加室内负样本或微调 CLRNet。

## 5. 室外测试、录包和离线复测

### 5.1 室外静止测试

第 3 节 IPM 已通过后，到室外道路保持静止：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_clrnet:=true line_detector:=clrnet \
  start_debug_viewers:=true enable_path_refine:=false
```

依次测试：中心线清晰、中心线遮挡但道路边缘清晰、两者都不可见、中心线恢复。
期望来源变化：

```text
center_line -> road_edge -> hold_last_valid -> no_visual_reference -> center_line
```

中心线清晰时，调试图中的紫色曲线应贴在线上，橙色目标点应位于线的另一侧；
状态中应有 `metric_calibrated=true` 和有效的 `target_ground_x/y`。

### 5.2 录制室外 rosbag

录包用于保存完整测试过程，至少覆盖上面的 4 段场景。建议使用系统 ROS Python：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
mkdir -p /home/nuc/lxw/neupan_ws/bags/road_test
rosbag record -O /home/nuc/lxw/neupan_ws/bags/road_test/vision_modes.bag \
  /camera/rgb/image_rect_color /camera/rgb/camera_info \
  /road_vision/debug_image /road_vision/center_line_mask \
  /road_vision/road_mask /road_vision/status /road_vision/lateral_error \
  /osm_global_path /osm_global_path_refined /scan /tf /tf_static
```

结束录包时只在录包终端按 `Ctrl+C`：

```bash
rosbag info /home/nuc/lxw/neupan_ws/bags/road_test/vision_modes.bag
```

### 5.3 离线复测

离线复测不启动 Astra 或 `image_proc`：

终端 1：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_camera:=false start_image_proc:=false \
  start_debug_viewers:=true enable_path_refine:=false
```

终端 2：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
rosparam set use_sim_time true
rosbag play --clock /home/nuc/lxw/neupan_ws/bags/road_test/vision_modes.bag
```

重点检查 `source` 切换、`hold_last_valid`、`no_visual_reference`、IPM 状态和误差
符号是否与实测方向一致。

## 6. 路径修正与 NeuPAN 联调

### 6.1 TF 和全局路径

先启动整车系统，再启动视觉节点；视觉节点仍保持路径修正关闭：

终端 1：

```bash
roslaunch neupan_ros full_system_run.launch \
  outdoor_mode:=true enable_rviz:=true
```

终端 2：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_debug_viewers:=true enable_path_refine:=false
```

确认相机外参：

```bash
rosrun tf tf_echo base_link camera_link
```

### 6.2 静止路径修正

视觉状态稳定后，车辆保持静止，才打开路径修正：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  start_debug_viewers:=true enable_path_refine:=true
```

在 RViz 同时显示 `/osm_global_path` 和 `/osm_global_path_refined`。确认来源切换
时路径连续，视觉失效时平滑回到原始路径，并且 `/osm_global_path` 不被修改。

### 6.3 低速运动

静止路径修正通过后，选择空旷路段低速直线运动并准备急停。确认修正方向、误差
限幅、视觉失效回退、Livox 避障和急停均正常后，再单独测试转弯。

## 7. 跟踪日志

launch 默认将每次启动的状态写入新的文件：

```text
/home/nuc/lxw/neupan_ws/tracker_logs/road_vision_status_YYYYMMDD_HHMMSS_microseconds.jsonl
```

每行一个 JSON，包含时间戳、`valid`、`source`、误差、CLRNet 状态、车道线侧别、
Hough 数量和拒绝原因。Ctrl-C 后节点关闭文件，下一次启动不会继续写旧文件。

查看最新日志：

```bash
ls -lt /home/nuc/lxw/neupan_ws/tracker_logs/road_vision_status_*.jsonl | head
tail -f "$(ls -t /home/nuc/lxw/neupan_ws/tracker_logs/road_vision_status_*.jsonl | head -1)"
```

指定其他目录：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  tracker_log_dir:=/绝对路径/tracker_logs
```

ROS 自身的 launch 日志仍由 ROS 保存到 `~/.ros/log`；如需同时保存完整终端输出：

```bash
roslaunch road_vision_refinement road_vision_debug.launch \
  2>&1 | tee tracker_logs/roslaunch_$(date +%Y%m%d_%H%M%S).log
```

## 8. 故障判断顺序

| 现象 | 先检查 |
|---|---|
| 没有图像窗口 | `start_debug_viewers:=true` |
| CLRNet 显示 `No module named torch/mmcv` | 使用 ROS 启动的同一个 Python 安装依赖 |
| CLRNet 出现下载进度 | 确认 checkpoint 存在；节点已关闭 backbone 二次下载 |
| 室内 `source=center_line` | `line_hough_count`、斜率拒绝、CLRNet 负样本 |
| 车道线左右跳变 | `line_reject_reason`、`center_line_x`、相机偏航和多线干扰 |
| `metric_calibrated=false` | IPM 文件路径、JSON 矩阵和图像分辨率 |
| `road_candidate_confidence=1.0` 但在室内 | 地面被低饱和度道路阈值选中，不代表真实道路 |
| 误差方向反了 | 检查 `base_link.y` 左正约定和 `lateral_error_m` 符号 |

## 9. 验收门槛

- IPM 独立点最大误差暂不超过 `0.25 m`；
- 室内无道路参考时最终为 `valid=false`、`source=none`；
- 室外静止测试完成 4 段来源切换且无明显误检；
- 录包可正常回放，离线状态与实时状态一致；
- 静止路径修正通过后才进行低速运动；
- 低速运动通过后才允许 NeuPAN 订阅 `/osm_global_path_refined`。
