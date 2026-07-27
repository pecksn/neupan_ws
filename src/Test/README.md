# test_motion_tools

Open-loop motion tests for recording straight-line and S-curve vehicle data.

These tests do not use global planning, local planning, or obstacle avoidance. They only publish `geometry_msgs/Twist` to `/neupan_cmd_vel`:

- `linear.x`: forward speed in m/s
- `angular.z`: Ackermann steering angle in rad

Start the base driver, GPS heading, and odometry/localization before running these tests.

## Straight Line

```bash
roslaunch test_motion_tools straight_record.launch speed:=0.4 duration:=20.0
```

## S Curve

```bash
roslaunch test_motion_tools s_curve_record.launch speed:=0.35 duration:=30.0 steer_amplitude_deg:=10.0 steer_period:=6.0
```

## Recorded Topics

The launch files start `rosbag record` automatically. Bag files are written under `/home/nuc/lxw/neupan_ws/tracker_logs`.

- `/gps/fix`: latitude, longitude, altitude
- `/heading`: ENU yaw as `QuaternionStamped`
- `/heading_deg`: G90 north-clockwise heading after install offset
- `/odometry/global`: global pose, ENU yaw, and velocity
- `/odometry/local`: local pose, yaw, and velocity
- `/neupan_cmd_vel`: commanded speed and steering angle
- `/open_loop_motion/status`: readable command status
- `/gps_guard/status`, `/lio_guard/status`, `/localization_safe`: localization health
- `/tf`, `/tf_static`: frame transforms
