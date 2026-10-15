# LXW 车辆仿真（ROS1 / Gazebo）

统一按资源类型组织，室内/局部与室外文件共用目录，通过 `outdoor_` 文件名区分。不在源码目录存放测试结果。

## 目录

```text
lxw_car_simulation/
├── launch/   启动入口与组件装配
├── scripts/  虚拟底盘、目标显示、室外测试工具
├── config/   局部仿真与 RVO 参数
├── maps/     室外 OSM 测试路网
├── worlds/   Gazebo 场景（含 outdoor_roads.world）
├── urdf/     车辆模型（含 outdoor_lxwcar.urdf.xacro）
├── rviz/     显示配置（含 outdoor_simulation.rviz）
├── meshes/   共用车辆 CAD 外观
├── models/   共用箱体、圆柱障碍物
└── README.md
```

| 文件 | 用途 |
|---|---|
| [launch/lxw_car_simulation.launch](launch/lxw_car_simulation.launch) | 原室内/局部仿真入口 |
| [launch/neupan_lxwcar_gazebo.launch](launch/neupan_lxwcar_gazebo.launch) | 上述入口调用的 Gazebo 与控制组件装配 |
| [launch/outdoor_simulation.launch](launch/outdoor_simulation.launch) | 室外 OSM 往返仿真装配，由测试脚本在独立 master 中启动 |
| [scripts/outdoor_simulation.py](scripts/outdoor_simulation.py) | 室外一键运行、传感器桥接、自动测试和录制 |
| [scripts/ackermann_sim_driver.py](scripts/ackermann_sim_driver.py) | 两种仿真共用的运动学虚拟底盘 |
| [scripts/publish_demo_goal.py](scripts/publish_demo_goal.py) | 原局部仿真的演示目标发布 |
| [scripts/gazebo_goal_marker.py](scripts/gazebo_goal_marker.py) | Gazebo 目标标记，无碰撞、不进入雷达回波 |

## ROS1 环境

先输入 `Neupan`、按 `e`，再执行：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
```

## 室外返程测试

使用 Gazebo 射线雷达、生产 OSM A*、[室外配置](../lxw_car/config/neupan_config_outdoor.yaml)、模型 `acker_robot_default_6` 与 NeuPAN 求解器。不启动实车驱动、串口、MID360、GNSS 或 GPU 接收。

三组自动测试并录制 rosbag：

```bash
rosrun neupan_ros outdoor_simulation.py run --scenario all --record-bag
```

只看宽路往返，打开 Gazebo 与 RViz：

```bash
rosrun neupan_ros outdoor_simulation.py run --scenario wide --gui --rviz --record-bag
```

入口自动创建独立 ROS master 和 Gazebo master，测试结束后关闭；控制与 TF 使用 `/outdoor_sim/*`，无需配置网卡 IP。不要把室外 launch 直接加载到正在控制实车的 master，避免 `/clock` 与 `/use_sim_time` 影响实车。

| 场景 | 路宽 | 过程与通过条件 |
|---|---|---|
| `wide` | 12 m | 前进约 10 m，再掉头返回；完成连接、到达目标、车体未越界 |
| `narrow` | 3 m | 同样下发返程目标；确认无安全连接，持续零速度至少 2 s、无漂移 |
| `unknown` | 12 m | 到第一目标后将回波置为 `inf`；不能把未知区域当成空路，拒绝掉头并停车 |

窄路/未知场景必须因 `no_safe_forward_connection` 停车，TF 失配或传感器未就绪不能冒充通过。雷达按原始采集时刻匹配 TF，不重写时间戳。到达阈值沿用生产配置的 0.5 m，并检查 Gazebo 车体与虚拟里程计最大位置差不超过 0.25 m。

室外车辆以后轴为参考点，轴距 0.67 m、规划外形 0.95×0.85 m、最小转弯半径 1.85 m；`base_link` 离地 0.20 m。OSM 路宽与 [worlds/outdoor_roads.world](worlds/outdoor_roads.world) 的物理边界一致，路网为 [maps/outdoor_test.osm](maps/outdoor_test.osm)。

### 结果

```text
/home/nuc/lxw/neupan_ws/videos/outdoor_simulation/日期_时间/
├── summary.md / summary.json
├── wide/
├── narrow/
└── unknown/
```

每个场景保存 `overview.mp4`（H.264 俯视轨迹）、`navigation.jsonl`、`result.json`、`console.log` 和 `ros_logs/`；加 `--record-bag` 后保存 `simulation.bag`。异常时 JSON/NPZ 也放在该场景目录。`--output-dir` 可覆盖保存位置。

视频不是摄像头画面：蓝色为 OSM 路线、绿色为实际接收参考线、粉色为局部规划、黄色为行驶轨迹。定位采用仿真真值，虚拟 GPS 仅提供接口；不验证真实 RTK、FAST-LIO、坡道、打滑或 HybridNets 靠右效果。

## 原室内 / 局部仿真

保持原启动方式，在 RViz 使用 `2D Nav Goal`：

```bash
roslaunch neupan_ros lxw_car_simulation.launch
```

自动发布演示目标：

```bash
roslaunch neupan_ros lxw_car_simulation.launch publish_demo_goal:=true
```

该入口不走室外 OSM A*，使用 [config/neupan_config_sim.yaml](config/neupan_config_sim.yaml)、模型 `acker_robot_default_2` 和原车辆模型；规划宽度/雷达自过滤宽度为 0.75 m。默认启用自适应避障参数，不要与室外 0.85 m 规划外形混用。原组件入口 `neupan_lxwcar_gazebo.launch` 仍可直接启动。

默认场景为 `worlds/lxw_car_obstacles.world`，不依赖 RVO。可用 `cad_visual:=false` 显示基础几何外观，用 `show_goal_marker:=false` 关闭目标标记。

### 动态障碍物（可选）

先确保 `rvo_ros`、`libmulti_obs.so` 及场景引用的模型可用：

```bash
roslaunch neupan_ros lxw_car_simulation.launch \
  dynamic_obstacles:=true \
  world_name:=$(rospack find neupan_ros)/example/lxw_car_simulation/worlds/gazebo_world_complex_20.world
```

默认障碍物最大速度 0.5 m/s，可用 `obstacle_max_speed:=0.8` 调整。随机目标范围为 x=[1.5,9.0]、y=[-4.0,4.0]，RVO 时间窗 2 s；随机模式连续 4 s 未移动 0.20 m 时重设目标。目标客户端最多等待服务 15 s，启动后应确认目标服务调用成功。过大的时间窗或过窄目标范围可能让人群停滞，应以模型实际位移判断运动，而非仅看配置速度。

## 共用模型与限制

- CAD 网格 `meshes/lxwcar_cad.stl` 来自毫米单位的 STEP，按 0.001 缩放，约 0.938×0.745×0.359 m，仅作外观；简化碰撞体与传感器仍独立生效。STL 不保留 SolidWorks 颜色，Gazebo 使用灰色材质。
- 实车 MID360 是 3D 点云再转 LaserScan；这里是较高的 2D 虚拟切片，障碍物需覆盖该高度。室内旧安装参数与室外模型分别保留，不把虚拟雷达高度当成真实安装标定。
- 共用虚拟底盘积分 Ackermann 模型，发布里程计、TF 和轮胎/转向关节状态，再通过 `/gazebo/set_model_state` 更新车体。它不是电机或轮胎动力学插件；默认指令超时 0.5 s 自动停止（`cmd_timeout`）。
- 仿真通过不代表实车验收。原室内仿真和 Hybrid A* 代码未因本次目录整理而改变。
