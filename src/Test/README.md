# test_motion_tools

`scripts/` 放人工运动测试和采集工具，`tests/` 放自动回归；两者不要混用。目录约定见 [功能包分类](../README.md)。

## 统一回归入口

自编的底盘限幅、控制切换、定位保护、驱动与定位组合、GNSS串口诊断、日志录包、相机路径及仿真几何测试均集中在 `tests/`。厂家/第三方自带测试和配套数据保持原位，不重复复制。这里只调整测试位置与导入路径，不修改运行节点或车辆参数。

进入 `Neupan`、按 `e`，在 ROS1 Noetic 的 `lxw` 环境执行：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s src/Test/tests -p 'test_*.py' -v
```

默认不启动底盘；需要独立 ROS 进程、C++ 搜索或规划器依赖的可选测试按各自条件跳过。单项可将 `-p` 换成对应文件名，例如 `test_serial_diagnostics.py`。无 ROS 环境不能直接执行全部回归。

10月21日末段右转/接收诊断整改后：Neupan容器lxw环境368项，360通过、8项可选测试跳过；真实启动入口导入、源码/devel模块一致性及三种launch配置解析通过。不包含实车驾驶验收。

Gazebo 整车测试需要显式运行 `scripts/test_command_gate.py` 或 `scripts/test_legacy_entry.py`，并指定 `--output-dir /home/nuc/lxw/neupan_ws/videos/<实验名>`，不混入默认单元测试。仿真模型和规划实现仍留在原仿真包，详见 [仿真报告](../neupan_ros/example/lxw_car_simulation/docs/planner_comparison.md)。

2026-10-21 迁移验证：隔离ROS1环境234项（230通过、4项可选测试跳过）；室外路径测试在具备曲线库/Tk依赖的主机验证73项（67通过、6项可选测试跳过）。合计307项、297通过、10跳过。临时ROS1基础镜像缺少曲线库的Tk导入依赖，因此采用分环境验证；不代表完整Gazebo或实车验收。

## GNSS测量时间回归与静止采集

室外日志复核工具 `scripts/verify_outdoor_reference.py` 可在lxw环境读取综合JSONL、按记录位姿重放稀疏/10 cm参考采样及偏离保护，并可通过`--bag`核对实际收到的话题；用`--output`指定新的JSON报告。它不会发布ROS消息，也不会预测修改后真实轨迹，不能替代带完整扫描的避障复现。

可选`--no-obstacle-window 15:57:44 15:57:50`对首个日志做去障碍物求解对照：校验模型哈希，使用近似求解开始位姿、记录参考索引及两种假定初始控制序列。它不是原始求解复现，不会运行车辆控制，不能证明当时具体障碍物或唯一根因。

实物静止采集（不启动底盘或规划）：先在ROS1 lxw环境运行 `roslaunch test_motion_tools sensor_time_check.launch`，另一个相同环境终端执行下列被动订阅命令。默认保存到 `tracker_logs/sensor_time_日期_时间/`，一分钟后采集器自动退出，随后Ctrl+C关闭传感器launch。无需开启车辆全系统。

```bash
python3 src/Test/scripts/check_sensor_time.py --seconds 60
```

此入口只运行GNSS、雷达及FAST-LIO，缺少LIO保护/EKF属于预期，不能当作完整定位链路验收。

`test_gnss_observation_time.py` 覆盖整数纳秒、GPS周/周内毫秒、接收机闰秒、GGA接收机日期、跨日/月/年、0/50/120/200 ms延迟、重复/乱序、时钟阶跃、CRC及未知时间系统；ROS1下还验证shadow不改变fix/GGA/heading/solution原时间契约。只验证旁路实现，不代表已将定位切到测量时标。

```bash
python3 -m unittest discover -s src/Test/tests -p 'test_gnss_observation_time.py' -v
```

2026-10-21 旁路验收：新增23项时间回归全部通过；隔离ROS1扩展回归共258项，254通过、4项可选测试跳过。室外launch仅做配置解析（shadow/未核时时基参数贯通），未启动实车；测量时标控制切换和T3动态验收仍未完成。

measurement软件补测：`test_gnss_measurement_contract.py` 检查真实驱动→桥的不同位置/航向历元、0/50/120/200 ms延迟下转弯插值、四种位置/定向质量组合、投影同历元去重与元数据晚到、无效时间/CRC拒绝、时钟阶跃禁止AUTO及保留续推预算。测试为假发布器，不允许据此声称实车切换已验收；Float测试使用合格的模拟协方差，未修改实车协方差阈值。

```bash
python3 -m unittest discover -s src/Test/tests -p 'test_gnss_measurement_contract.py' -v
```

2026-10-21最终软件验证：真实ROS1/lxw环境统一回归358项，350通过、8项可选测试跳过；另一次隔离ROS master进程测试通过（局部位姿延迟120 ms）。底盘包编译、devel共享模块及室外原命令三端measurement/auto参数解析通过，禁止的旧全局EKF组合在启动前拒绝。未启动实车控制，未取得室外有效时间或动态验收。

`test_gnss_auto_start.py`新增启动UNKNOWN→有效GNSS时间无需重启/改参即可放行、两秒连续窗口、NUC未同步、重复帧、启动钟跳重新等待、运行钟跳/失同步锁存保护等测试。真实NUC只读内核查询返回已同步；测试不修改系统时间，不把自动一致性门控称为硬同步证明。

```bash
python3 -m unittest discover -s src/Test/tests -p 'test_gnss_auto_start.py' -v
```

本机lxw没有`defusedxml`，启动配置测试需要它；ROS系统环境已安装。本次完整回归仅给当前测试进程追加系统库搜索路径，没有安装库或改conda配置，可复用：

```bash
python3 -c 'import sys,unittest; sys.path.append("/usr/lib/python3/dist-packages"); suite=unittest.defaultTestLoader.discover("src/Test/tests", pattern="test_*.py"); result=unittest.TextTestRunner(verbosity=1).run(suite); sys.exit(not result.wasSuccessful())'
```

## 室外返程路径衔接回归（不驱动车辆）

在 ROS1 的 `lxw` 环境运行；覆盖反向目标、前进连接、实车 0.4 m 扫描过滤、近场缓冲区未验证标记、车身进入盲区拒绝、缓冲区障碍、采集 TF 延迟重试、空间/观测不足停车、靠右路径切换及室内配置隔离。规划与路径策略归属 `osm_map_handler`，另检查迁移后的引用及独立 3 s 心跳回退；NeuPAN 仅保留控制接入。

另覆盖稀疏路线段距离、日志中的 1.46 m 同向起步、仅 ≥135° 反向路线触发掉头，以及普通转弯/大偏移/目标在身后但路线未反向时保留原跟踪、不新增道路或扫描准入门槛。隔离 ROS1 测试检查偏移起步确实生成非零测试指令，3 m 偏移和 90° 转弯不进入搜索，反向且观测不足时持续零速度。

```bash
cd /home/nuc/lxw/neupan_ws
python src/Test/tests/test_outdoor_start_connection.py -v
```

编译后加两个开关可同时运行真实 C++ 搜索与隔离 ROS1 入口测试：独立临时 master、原室内 Hybrid A*、假传感器和测试速度话题，不启动底盘驱动。包含异步取消、超时停车、迭代耗尽不返回半条路径、最新扫描复查及车辆移动拒绝。

```bash
NEUPAN_CPP_SMOKE=1 NEUPAN_ROS_SMOKE=1 python src/Test/tests/test_outdoor_start_connection.py -v
```

## 相机路径几何回归（不驱动车辆）

从旧目录迁入的测试使用当前 `osm_map_handler` 模块，不再引用已删除的底盘包旧脚本。需要 `lxw` 环境中的 OpenCV / NumPy；默认仅运行几何、缓存、投影和回退测试，不启动 ROS。

```bash
python src/Test/tests/test_camera_right_refinement.py
```

`--ros` 是历史影子联调入口，要求预先启动独立 `127.0.0.1:11319` master 和影子节点，并保留原 20261008 测试 bag；不会自动启动这些依赖，禁止指向实车 master。

## NeuPAN录包障碍复核（不驱动车辆）

`verify_neupan_obstacles.py` 在Neupan容器的ROS1、lxw环境中，读取bag和对应综合日志，校验模型哈希，对比完整/降采样障碍点、记录轨迹与当前停车判据，并对少量样本做条件重算。它不启动节点、发布速度或打开串口。原回调时序和求解器warm-start未全部记录，因此不是逐步精确重放，也不等于实车碰撞验收。

```bash
cd /home/nuc/lxw/neupan_ws
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 \
  src/Test/scripts/verify_neupan_obstacles.py \
  --bag rosbags/rosbag20261020_160851/hybridnets_vehicle_data_20261020_160851_0.bag \
  --log tracker_logs/system_run_full_20261020_160844.jsonl \
  --output videos/neupan_obstacle_check_20261020_160851
```

输出JSON及扫描/预测车体叠加图，不更改原bag、模型或业务配置。

## 左右电机映射测试

MCU 命令 `CMD,<throttle>,<steering>` 只有统一油门，不能独立驱动左右电机。
在 Neupan 容器中，可以通过转动架空车轮并测量 `LT/RT` 扭矩响应来判断映射：

```bash
cd /home/nuc/lxw/neupan_ws
python3 src/Test/scripts/motor_side_mapping_test.py
```

运行前停止 `robot_base_driver`，并将两个驱动轮可靠架空。程序会保持零油门，依次
提示转动物理左轮和物理右轮，最后输出左右映射是否颠倒。

Open-loop motion tests for recording straight-line and S-curve vehicle data.

These tests do not use global planning, local planning, or obstacle avoidance. They only publish `geometry_msgs/Twist` to `/neupan_cmd_vel`:

- `linear.x`: forward speed in m/s
- `angular.z`: Ackermann steering angle in rad

The test launch starts `robot_base_driver` and the G90 GPS driver by default, so running it alone can move the car and record GPS/heading. It does not start global planning, local planning, NeuPAN, or TEB.

If another launch has already started `robot_base_driver` or the G90 GPS driver, set `start_base_driver:=false` or `start_gps_driver:=false` to avoid two nodes opening the same serial port.

## Straight Line

```bash
roslaunch test_motion_tools straight_record.launch speed:=0.4 duration:=20.0
```

## S Curve

```bash
roslaunch test_motion_tools s_curve_record.launch \
  speed:=0.4 \
  duration:=45.0 \
  steer_amplitude_deg:=18.0 \
  steer_period:=7.0 \
  rate:=1.0
```

Before sending non-zero speed, the motion node waits for valid `/gps/fix`, `/heading`, and at least one `/neupan_cmd_vel` subscriber by default. It then waits `start_delay:=2.0` seconds before movement. `ramp_time` defaults to `0.0`, so the recorded `speed` is the constant upper-computer commanded speed.

## Rosbag Topic

The launch files start `rosbag record` automatically. Bag files are grouped by
Beijing time under `/home/nuc/lxw/neupan_ws/rosbags/rosbagYYYYMMDD_HHMMSS/`; timestamped
filenames sort in recording order.

The rosbag records only one compact topic:

```text
/open_loop_motion/packet  test_motion_tools/MotionPacket
```

`MotionPacket` contains only these fields:

- `int32 lat`: latitude x 1e7
- `int32 lon`: longitude x 1e7
- `int16 alt`: altitude x 10, in decimeters
- `int16 yaw`: ENU yaw x 100, in centi-degrees
- `int8 speed`: upper-computer commanded speed x 100, in cm/s
- `int8 steer`: commanded steering angle, in degrees

By default, `/open_loop_motion/packet` is not published until `/gps/fix`, heading (`/heading` or `/heading_deg`), and `/neupan_cmd_vel` have all been received. If the compact bag has no packet messages, check the GPS serial port, or set `require_packet_sources:=false` for command-only tests.

Packets with `speed == 0` are skipped by default, so stop commands before/after the motion are not written to rosbag or JSONL.

## Synchronized JSONL Log

The launch also starts `open_loop_data_logger.py` by default. It samples the latest values at 20 Hz and writes one synchronized JSONL log:

```text
/home/nuc/lxw/neupan_ws/tracker_logs/open_loop_<mode>_sync_YYYYmmdd_HHMMSS.jsonl
```

Each JSONL line contains only the same six fields as the compact rosbag packet:

```json
{"lat":230398123,"lon":1133900456,"alt":12,"yaw":8560,"speed":30,"steer":0}
```

## Synchronized JSONL Fields

- `lat`: latitude x 1e7
- `lon`: longitude x 1e7
- `alt`: altitude x 10
- `yaw`: ENU yaw x 100
- `speed`: upper-computer commanded speed x 100
- `steer`: commanded steering angle in degrees
