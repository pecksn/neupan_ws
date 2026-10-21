# robot_base_driver

底盘、定位保护、控制选择和日志节点统一放 `scripts/`；回归统一放 [`../Test/tests`](../Test/tests)，消息、参数和启动文件仍分别放 `msg/`、`config/`、`launch/`。`host_process_monitor.py` 继续在主机执行，不随分类改成容器内采集。节点文件名与原启动命令不变，不新增无用途的 `src/` 或 `python/`；见 [功能包分类](../README.md)。

## 无 Fixed 的运行中定位

整车室外入口默认measurement＋auto，包的独立入口仍保留shadow。`gnss_timebase.py`共用于投影和定位桥；auto允许无有效GNSS时间时先启动等待，收到连续合格时间元数据才放行，不把auto当作预先确认true。运行后时钟异常需停车重启，原GNSS质量和续推预算不放宽。具体见 [时间整改](../../PROJECT_ISSUES_CN.md#22-测量时间对齐软件链路已实现实车切换待验收)。

`global_odom_bridge.py` 中独立的位置校正策略允许初始化后的合格 Float／差分位置慢速纠偏，不采用未固定 GNSS 航向。连续新样本与 LIO 一致、完成纠偏后才恢复续推预算，非收到一帧就重置。完整条件见 [室外导航](../../GDUT室外导航.md#运行中不要求-fixed-的位置校正2026-10-16)。

质量必须属于本次观测：GPS投影/fix/GGA源时间完全一致，完整航向校正要求航向/解状态的整数 `stamp_ns` 完全一致；缺少该帧位置质量或新航向自己的质量时有界等待，不借相邻帧、不提前消费。位置与定向状态独立：初始化后启用原运行策略时，合格位置-only候选不依赖有效航向，定向非Fixed不发布有效航向仍可按原窗口续期且yaw不变。最新定向失效独立保存在 `/gps_guard/status.latest_heading_health`，历史Fixed缓存不得遮盖；收到失效后退出完整Fixed跟踪，旧/重复Fixed不能恢复它，也不能用新Fixed回填旧Float。

同报文绑定要求更新并重启 `wheeltec_gps_driver`：Fixed模式改从CRC校验通过的PVTSLNA直接发布航向及对应解状态，HDT不再借旧状态发布。设备已由用户确认配置10 Hz，不能因旧ROS记录1 Hz就要求提频；应先看 `/gnss/serial_diagnostics` 及实际流量，不放宽0.5 s时效。旧bag缺少 `stamp_ns` 时不获准完整航向校正。

全局校正以当前车位置为锚点，联合更新平移/yaw，避免远离odom原点后航向恢复引入米级跳动。GPS位置和航向分别按采集时间插值局部位姿，恢复窗口也在同一锚点比较。每路GNSS观测连同采集/接收时间及质量信息缓存最多128条；选择最新可配对且未处理的帧，不能用新质量覆盖旧观测，也不倒序重放。局部位姿迟到时，短暂等待保留当前状态、变换和稳定窗口；私有参数 `~pair_wait_timeout_sec` 默认0.5 s（不超过原数据/航向时限），不能被连续未配对的新消息延长。超时后走原降级/续推预算，从等待开始计时和计位移。真实质量失效、过期或LIO异常不享受此宽限。

`/gps_guard/status` 的 `source_pairing`、`recovery_window`、`alignment_correction`、误差有效性和 `observation_buffer` 随原综合JSONL/rosbag记录；后者区分 `pair_wait`、`pair_timeout`、`selected`，记录选中帧的源时间、年龄和原始质量。输入时间水位仅在完整配对与质量检查通过后推进，等待中的新帧不能把旧待配对帧挤成乱序。

持续、稳定、完整质量检查通过的Fixed位置冲突会锁存 `alignment_conflict` 并禁止AUTO；默认位置门限3 m，持续时间跟随恢复窗口（全系统3 s、节点默认4 s）。单帧Float不触发此新锁存，原非Fixed策略和续推预算保留。触发后应停车核查、重启整套定位/导航、确认位置朝向并重新下发路径，不能行驶中只重启桥或继续旧路径。实现、测试边界和待实车项目见 [P1问题记录](../../PROJECT_ISSUES_CN.md#3-p1长距离运行后gps恢复校正导致地图位置偏移)。

NeuPAN 全系统默认 `gps_guard_allow_nonfixed_runtime:=true`；本包 `navigation_core.launch` 及节点默认 false，以保留其他调用者的原行为。100 m / 600 s 无可信位置续推限制、LIO 异常停车与启动固定航向要求保留。`/gps_guard/status` 的 `runtime_position` 记录质量检查与预算恢复；新模式为 `GPS_POSITION_TRACKING`，不是 RTK 高精度状态。

不启动底盘的回归：

```bash
python3 -m unittest discover -s src/Test/tests -p 'test_*.py' -q
```

Fixed、粗位置启动、运行中校正统一验证源时间；重复 GPS 样本不重复纠偏或推进窗口。真实Fixed失效或配对超时进入续推预算，配对等待不能绕过限额。`/gps_guard/status.last_processed_gps_source_ns` 可核对最后处理的真实样本。隔离ROS进程测试可设置 `GPS_RUNTIME_ROS_SMOKE=1 GPS_RUNTIME_LOCAL_DELAY_SEC=.12`，验证持续120 ms局部位姿延迟；不要将该测试接入实车ROS master。

编译后设置 `GPS_RUNTIME_ROS_SMOKE=1` 可额外运行独立临时 ROS master 和假消息联调：Fixed 正常初始化→时间配对持续失配触发保护→合格 Float 恢复并超过测试预算继续运行→Fixed 恢复→重复旧 Fixed 源数据再次触发保护；不连接车辆 master。2026-10-16 本包 61 项、OSM/相机/NeuPAN 入口 101 项全部通过，详见[自测记录](../../videos/localization_runtime/20261016_nonfixed/summary.md)。

## 自动模式定位安全信号超时

`control_mode_mux.py` 在启用 `enable_auto_localization_safety_stop` 时，必须持续收到 `/localization_safe`。默认 `~localization_safety_timeout=1.0` 秒，以单调接收时钟计时：未收到、超时或收到 false 均输出零自动速度。收到新鲜 true 后仍检查原自动指令超时；手动遥控及未启用本项的调用者不受影响，不检查 GPS 是否 Fixed。

NeuPAN 全系统已传入同名参数 `localization_safety_timeout`，原启动命令即可生效。`/vehicle/control_mode/status` 每 0.5 秒发布 `localization_safe`（包含超时判断的有效值）、`localization_safe_reported`（收到的原值）、`localization_safety_reason/age_sec/timeout_sec`，随原 JSONL 和 rosbag 记录；不同拦截原因在终端 WARN 提示。原 Bool 话题没有源时间，本项仅判断消息是否持续到达，不能代替上游定位数据质量检查。

回归在 `tests/test_control_mode_mux.py`，17 项消息级测试加 1 项隔离 ROS 联调；2026-10-17 验证规划器连续发速度时，安全信号断流约 1.04 秒后选定速度归零，并验证手动接管及有效信号恢复。不启动底盘：

```bash
MUX_SAFETY_ROS_SMOKE=1 python3 -m unittest discover \
  -s src/Test/tests -p 'test_control_mode_mux.py' -q
```

## 最终速度输入保护（任务02）

`base_driver_node.py` 在回调及最终串口发送层拒绝非有限速度/转角：任一实际控制字段为NaN/Inf时，立即发送 `CMD,0.00,0.00`，不能保留上一条运动指令。有限大值按独立正反向软件上限及原±25°转角限制处理；保留0.05 m/s死区、停车回正、倒车和电机仅报警行为，不新增变化率限制。

`max_forward_speed_mps`、`max_reverse_speed_mps` 默认均为1.5 m/s，`wheel_radius` 默认0.1275 m；三者须有限且大于0，换算后的轮速限值也须有限且大于0，否则在打开串口前报错。默认速度来自当前室外规划的±1.5软件边界，不代表硬件安全极速。串口保持轮速rad/s、转角deg，限值附近向内量化避免0.01 rad/s四舍五入越界。告警2秒节流。

三个参数可从 `driver.launch`、`navigation_core.launch` 或 `full_system_run.launch` 覆盖；旧实车入口继承默认保护，不依赖mux是否启用。详见[统一问题记录](../../PROJECT_ISSUES_CN.md)。任务03尚未实施，不影响既有MCU失联停车或电机报警策略。

模拟串口回归，不打开真实串口或启动底盘：

```bash
/usr/bin/python3 -m unittest discover \
  -s src/Test/tests -p 'test_base_driver_command_limits.py' -q
```

## 控制审计与综合日志

GPS串口诊断 `/gnss/serial_diagnostics` 默认1 Hz，完整统计及实际模块路径/哈希进入原综合JSONL的 `gnss_serial_diagnostics_event`，原10 Hz状态仅保留简要摘要。三份录包配置也包含该非必需话题。它统计主机读取、解码、识别、解析和Publisher调用，不能当作接收机物理输出或ROS交付确认；未保存原始字节全文。GPS驱动和综合日志节点重启后生效。

底盘在每次串口发送尝试后发布 `/vehicle/command_audit`（`std_msgs/String` JSON）：原始输入、限幅/死区/非法值/定位拦截原因、准确的协议字符串、量化后轮速rad/s和转角deg、换算车速、限值、写入字节数及错误。`write_status` 为 `written`、`short_write`、`write_error` 或 `port_unavailable`；仅 `written` 表示串口API返回完整字节数，**不代表MCU已执行**。没有新增执行反馈或控制联锁。审计发布为有界队列、尽力而为，异常不阻断原控制发送。

`system_run_logger.py` 将该话题写为 `command_audit_event`，并逐消息记录 `/cmd_vel_remote` 的 `remote_command_event` 和 `/vehicle/control_mode` 的 `mode_request_event`（含非法模式请求）。三者也进入原10 Hz状态采样；仍写入同一份 `tracker_logs/system_run_full_*.jsonl`，不新增另一套日志文件。事件外层 `stamp` 是NUC ROS接收时间、`wall_stamp` 是系统时间；无Header的原始遥控/模式请求没有采集时间，不能将接收时间当作源时间。驱动审计的 `data.stamp` 为驱动发送处理开始时刻。

全系统开启原综合日志即可，rosbag仍需另开；三份录包配置均包含上述话题。现有运行中的进程不会热更新，安全停车后重新启动相应系统才应用新代码。

综合JSONL在统一写入边界将非有限数递归保留为字符串 `"nan"`、`"inf"`、`"-inf"`，不改为零、不修改控制数据，再用严格JSON序列化；自动事件、selected采样及嵌套诊断同样生效。其他不可序列化对象会告警，本条写入返回失败，后续记录仍可继续。

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
