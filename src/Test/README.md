# test_motion_tools

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

The launch files start `rosbag record` automatically. Bag files are written under `/home/nuc/lxw/neupan_ws/tracker_logs`.

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
