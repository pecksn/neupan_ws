# robot_base_driver

底盘、定位保护、控制选择和日志节点统一放 `scripts/`；回归放 `tests/`，消息、参数和启动文件仍分别放 `msg/`、`config/`、`launch/`。`host_process_monitor.py` 继续在主机执行，不随分类改成容器内采集。节点文件名与原启动命令不变，不新增无用途的 `src/` 或 `python/`；见 [功能包分类](../README.md)。

## 无 Fixed 的运行中定位

`global_odom_bridge.py` 中独立的位置校正策略允许初始化后的合格 Float／差分位置慢速纠偏，不采用未固定 GNSS 航向。连续新样本与 LIO 一致、完成纠偏后才恢复续推预算，非收到一帧就重置。完整条件见 [室外导航](../../GDUT室外导航.md#运行中不要求-fixed-的位置校正2026-10-16)。

NeuPAN 全系统默认 `gps_guard_allow_nonfixed_runtime:=true`；本包 `navigation_core.launch` 及节点默认 false，以保留其他调用者的原行为。100 m / 600 s 无可信位置续推限制、LIO 异常停车与启动固定航向要求保留。`/gps_guard/status` 的 `runtime_position` 记录质量检查与预算恢复；新模式为 `GPS_POSITION_TRACKING`，不是 RTK 高精度状态。

不启动底盘的回归：

```bash
python3 -m unittest discover -s src/robot_base_driver/tests -p 'test_*.py' -q
```

Fixed、粗位置启动、运行中校正统一验证源时间；重复 GPS 样本不重复纠偏或推进窗口。Fixed 失效时先启动续推预算，配对等待不能绕过限额；短暂配对等待不会清空已有稳定窗口。`/gps_guard/status.last_processed_gps_source_ns` 可核对最后处理的真实样本。

编译后设置 `GPS_RUNTIME_ROS_SMOKE=1` 可额外运行独立临时 ROS master 和假消息联调：Fixed 正常初始化→时间配对持续失配触发保护→合格 Float 恢复并超过测试预算继续运行→Fixed 恢复→重复旧 Fixed 源数据再次触发保护；不连接车辆 master。2026-10-16 本包 61 项、OSM/相机/NeuPAN 入口 101 项全部通过，详见[自测记录](../../videos/localization_runtime/20261016_nonfixed/summary.md)。

## 自动模式定位安全信号超时

`control_mode_mux.py` 在启用 `enable_auto_localization_safety_stop` 时，必须持续收到 `/localization_safe`。默认 `~localization_safety_timeout=1.0` 秒，以单调接收时钟计时：未收到、超时或收到 false 均输出零自动速度。收到新鲜 true 后仍检查原自动指令超时；手动遥控及未启用本项的调用者不受影响，不检查 GPS 是否 Fixed。

NeuPAN 全系统已传入同名参数 `localization_safety_timeout`，原启动命令即可生效。`/vehicle/control_mode/status` 每 0.5 秒发布 `localization_safe`（包含超时判断的有效值）、`localization_safe_reported`（收到的原值）、`localization_safety_reason/age_sec/timeout_sec`，随原 JSONL 和 rosbag 记录；不同拦截原因在终端 WARN 提示。原 Bool 话题没有源时间，本项仅判断消息是否持续到达，不能代替上游定位数据质量检查。

回归在 `tests/test_control_mode_mux.py`，17 项消息级测试加 1 项隔离 ROS 联调；2026-10-17 验证规划器连续发速度时，安全信号断流约 1.04 秒后选定速度归零，并验证手动接管及有效信号恢复。不启动底盘：

```bash
MUX_SAFETY_ROS_SMOKE=1 python3 -m unittest discover \
  -s src/robot_base_driver/tests -p 'test_control_mode_mux.py' -q
```

## 电机遥测与报警

下位机可按行发送：

```text
MOTOR,LT:<左扭矩>,RT:<右扭矩>,LMOS:<左MOS温度>,LCOIL:<左线圈温度>,RMOS:<右MOS温度>,RCOIL:<右线圈温度>,LSTATE:<左状态码>(<含义>),RSTATE:<右状态码>(<含义>)\r\n
```

节点发布：

- `/motor/status` (`robot_base_driver/MotorStatus`)：六项实时值、报警状态及原因。
- `/motor/alarm` (`std_msgs/Bool`)：锁存的总报警状态。
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)：供 `rqt_robot_monitor` 使用。

针对 DM-S3519-1EC，默认预警阈值为扭矩绝对值 `7.0 Nm`、MOS `100 C`、
线圈 `90 C`；扭矩回差为 `0.5 Nm`，温度回差为 `5 C`。

```bash
roslaunch robot_base_driver driver.launch \
  motor_torque_limit:=7.0 \
  motor_mos_temp_limit:=100.0 \
  motor_coil_temp_limit:=90.0 \
  motor_torque_hysteresis:=0.5 \
  motor_temp_hysteresis:=5.0
```

查看实时状态：

```bash
rostopic echo /motor/status
rostopic echo /motor/alarm
```

状态码：`0 DISABLED`、`1 ENABLED`、`8 OVER_VOLTAGE`、`9 UNDER_VOLTAGE`、
`A OVER_CURRENT`、`B MOS_OVER_TEMP`、`C COIL_OVER_TEMP`、`D COMM_LOST`、
`E OVERLOAD`；其他值显示为 `UNKNOWN`。任一侧状态不为 `1` 时，总报警会包含
明确的 `LEFT state ...` 或 `RIGHT state ...` 原因。

启用 `system_run_logger.py` 时，每条 JSONL sample 的 `data.motor` 会记录六项实时值、
报警状态、报警原因以及该数据的接收时延。
