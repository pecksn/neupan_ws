# robot_base_driver

## 电机遥测与报警

下位机可按行发送：

```text
MOTOR,左电机扭矩,右电机扭矩,左MOS温度,左线圈温度,右MOS温度,右线圈温度\r\n
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

启用 `system_run_logger.py` 时，每条 JSONL sample 的 `data.motor` 会记录六项实时值、
报警状态、报警原因以及该数据的接收时延。
