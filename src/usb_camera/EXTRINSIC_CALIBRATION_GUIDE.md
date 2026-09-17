# USB Camera 与 Livox MID360 外参标定指南

## 1. 当前项目状态

### 相机内参

相机已经完成 `1920x1080` 内参标定，项目内标定文件为：

```text
src/usb_camera/config/usb_camera.yaml
```

启动配置通过以下 URL 加载该文件：

```yaml
camera_info_url: "package://usb_camera/config/usb_camera.yaml"
```

主要参数：

```text
fx = 1102.837297249009
fy = 1104.805244636163
cx = 929.1071103499039
cy = 716.3749386027972

k1 = -0.3440482313019139
k2 =  0.08610560344908336
p1 = -0.003154023779031887
p2 = -0.004847752761592449
k3 =  0
```

这些参数对应旋转 180 度后的 `/camera/image_raw`。不要与
`/camera/image_raw_unrotated` 混用。

### 相机外参

相机目前没有以下物理外参：

```text
base_link -> camera_optical_frame
livox_frame -> camera_optical_frame
```

因此项目目前不知道相机相对车辆的位置、安装高度和安装角度。
`camera_frame_id: camera_optical_frame` 只是图像消息中的坐标系名称，
并不代表已经存在 TF 外参。

### MID360 外参和高度

实车启动文件当前包含：

```xml
<node pkg="tf" type="static_transform_publisher"
      name="base_to_livox"
      args="0.1 0 0.25 0 0 0 base_link livox_frame 100"/>
```

所以代码当前假定 MID360 相对 `base_link`：

```text
x = 0.10 m（向前）
y = 0.00 m（向左）
z = 0.35 m（向上，已按雷达升高 10 cm 更新）
roll = pitch = yaw = 0
```

也就是说，配置中的雷达高度是相对 `base_link` 原点向上 `0.35 m`，
不是必然等于雷达距离地面的真实高度。只有当 `base_link` 位于地面时，
它才可以解释为离地 35 cm。该启动文件注释也说明这是一组需要根据
实车尺寸调整的机械安装外参。

同一组近似值还用于：

```text
base_link -> body
```

这会影响 FAST-LIO 点云转换和 `pointcloud_to_laserscan` 的高度过滤，
因此应当实车测量后统一修改。

## 2. 需要得到的最终 TF 链

推荐使用：

```text
base_link
└── livox_frame
    └── camera_optical_frame
```

其中：

1. `base_link -> livox_frame` 通过实车机械测量获得。
2. `livox_frame -> camera_optical_frame` 通过相机—雷达联合标定获得。
3. 相机相对车体的外参由两段变换组合得到。

注意：FAST-LIO 的 `camera_init` 是其全局初始化坐标系，与实体摄像头
没有关系，不要将相机挂到 `camera_init` 下。

## 3. 标定前的机械测量

先在车体上定义 `base_link` 原点，并按 ROS REP-103 方向测量：

```text
x：车辆前方为正
y：车辆左方为正
z：车辆上方为正
```

记录 MID360 中心相对 `base_link` 的：

```text
lidar_x, lidar_y, lidar_z
```

同时记录相机光心的大致位置：

```text
camera_x, camera_y, camera_z
```

机械测量值用于检查优化结果和提供外参初值，但不能替代相机—雷达
旋转外参标定。

## 4. 外参标定板

20 mm 小棋盘适合相机内参，不适合 MID360 外参。建议准备：

- 约 `1.0 x 1.2 m` 或更大的硬质平板；
- 平整、不弯曲、表面哑光；
- 四个外角在相机图像和点云中都能清晰定位；
- 可以在板面粘贴大棋盘或 AprilTag，辅助图像角点识别。

相机、雷达和车辆必须刚性固定，整个标定过程不能改变相对位置。

## 5. 启动传感器

进入 Neupan ROS 1 环境：

```bash
Neupan
# 按 e
conda activate lxw
cd /home/nuc/lxw/neupan_ws
source devel/setup.bash
```

终端 1 启动旋转后的相机：

```bash
roslaunch usb_camera camera.launch
```

标定必须使用：

```text
/camera/image_raw
/camera/camera_info
```

不要使用未旋转话题 `/camera/image_raw_unrotated`。

终端 2 让 MID360 发布标准 `PointCloud2`：

```bash
roslaunch livox_ros_driver2 rviz_MID360.launch \
  xfer_format:=2 rviz_enable:=true
```

确认：

```bash
rostopic type /livox/lidar
```

预期输出：

```text
sensor_msgs/PointCloud2
```

标定时不要同时运行完整导航系统或第二个 MID360 驱动，否则设备或
网络端口会冲突。

## 6. 采集数据

建立目录：

```bash
mkdir -p /home/nuc/lxw/neupan_ws/calibration_data
```

每个姿态录制约 10 秒：

```bash
rosbag record \
  -O /home/nuc/lxw/neupan_ws/calibration_data/pose_01.bag \
  /livox/lidar \
  /camera/image_raw \
  /camera/camera_info
```

建议采集至少 15 组：

- 左、右、上、下和画面中央；
- 近、中、远距离，建议约 2 至 4 米；
- 向左、向右、向上、向下倾斜；
- 每组采集过程中标定板和车辆完全静止。

## 7. 建立二维—三维对应关系

每个姿态提取平板四个角：

```text
图像坐标：(u, v)
雷达坐标：(x, y, z)
```

统一采用相同顺序，例如：

```text
左上 -> 左下 -> 右下 -> 右上
```

一组顺序错误就会显著破坏优化结果。建议先将每组静态点云累积为 PCD，
再在 PCL/RViz 工具中选择三维角点。

## 8. 外参求解

固定当前相机内参 `K` 和畸变 `D`，仅优化：

```text
R_camera_lidar
t_camera_lidar
```

求解关系通常写成：

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar
```

即输出 `T_camera_lidar`。如果要在 TF 树中发布：

```text
livox_frame -> camera_optical_frame
```

通常需要先计算逆矩阵：

```text
T_lidar_camera = inverse(T_camera_lidar)
```

必须检查所用求解器的矩阵定义，不能只根据变量名称判断方向。

## 9. 写入 ROS TF

得到并验证 `livox_frame -> camera_optical_frame` 的平移和欧拉角后，可以
加入静态 TF：

```xml
<node pkg="tf" type="static_transform_publisher"
      name="livox_to_camera"
      args="x y z yaw pitch roll livox_frame camera_optical_frame 100"/>
```

ROS 1 `tf/static_transform_publisher` 的角度参数顺序是：

```text
yaw pitch roll
```

单位是弧度。

## 10. 结果验证

必须将 MID360 点云投影到旋转后的相机图像上，检查：

- 墙面点落在墙面内；
- 门框、桌沿、立柱等深度突变边缘与图像边缘重合；
- 近处和远处都能对齐；
- 多个独立场景都能对齐，而不只标定板场景；
- 静止时投影稳定，运动时再检查时间同步误差。

如果近处误差大而远处较好，优先检查平移；如果整个画面误差随方向
变化，优先检查旋转和矩阵方向；如果运动时错位而静止时正确，则需要
继续做时间同步，而不是重新调整空间外参。

## 11. 当前项目仍需完成的事项

- 复核已更新的 `base_link -> livox_frame = 0.1, 0, 0.35` 机械值；
- 求解 `livox_frame -> camera_optical_frame`；
- 增加静态 TF；
- 增加点云投影验证节点；
- 如需车辆运动中的图像点云融合，再处理相机和 MID360 时间同步；
- 将相机话题接入实际感知模块。目前导航、避障和路沿检测均未消费相机。

机械测量得到的相机初值保存在：

```text
src/usb_camera/config/livox_camera_extrinsics.yaml
```

该文件标记为 `initial_guess`，在完成点云投影验证之前不作为最终静态 TF
发布。
