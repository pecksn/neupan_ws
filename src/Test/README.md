# test_motion_tools

`scripts/` 放人工运动测试和采集工具，`tests/` 放自动回归；两者不要混用。目录约定见 [功能包分类](../README.md)。

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
