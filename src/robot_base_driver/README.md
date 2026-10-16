# robot_base_driver

底盘、定位保护、控制选择和日志节点统一放 `scripts/`；回归放 `tests/`，消息、参数和启动文件仍分别放 `msg/`、`config/`、`launch/`。`host_process_monitor.py` 继续在主机执行，不随分类改成容器内采集。节点文件名与原启动命令不变，不新增无用途的 `src/` 或 `python/`；见 [功能包分类](../README.md)。

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
