# lxw_car_simulation

Gazebo simulation entry for the LXW Ackermann car with NeuPAN.

## What is included

- `urdf/`: primitive-shape Ackermann URDF/Xacro for Gazebo.
- `launch/neupan_lxwcar_gazebo.launch`: starts Gazebo, spawns the car, starts `robot_state_publisher`, starts `neupan_node.py`, and optionally starts RViz/demo goal.
- `worlds/gazebo_world_complex_20.world`: copied from the official `example/gazebo_limo` complex obstacle world.
- `worlds/lxw_car_obstacles.world`: simple static-obstacle fallback world.
- `models/`: reusable fallback box and cylinder obstacle models.
- `config/neupan_config_sim.yaml`: NeuPAN simulation config aligned with the LXW car geometry.
- `config/rvo_config_complex.yaml`: RVO dynamic obstacle config copied from the official Gazebo LIMO example.
- `rviz/lxw_car_simulation.rviz`: RViz view for `/scan`, TF, robot model, and NeuPAN markers/paths.
- `scripts/publish_demo_goal.py`: optional latched goal publisher for quick smoke tests.

## Sensor And Avoidance Notes

The simulation keeps the real MID360 mounting frame at `livox_frame = 0.1 0 0.25` relative to `base_link`, matching `example/lxw_car/launch/full_system_run.launch`. Because Gazebo's `ray` sensor is only a 2D slice while the real car uses MID360 point cloud plus `pointcloud_to_laserscan`, the simulated `laser_link` is placed higher as a virtual scan slice. The obstacle worlds add taller collision cylinders so that this slice is actually visible in `/scan`.

NeuPAN parameters in `config/neupan_config_sim.yaml` follow the real outdoor baseline. The launch enables adaptive adjustment by default; when an obstacle enters the front sector, it blends toward the avoid profile from `full_system_run.launch`.

## Run

```bash
cd /home/nuc/lxw/neupan_ws
catkin_make
source devel/setup.bash
roslaunch neupan_ros example/lxw_car_simulation/launch/neupan_lxwcar_gazebo.launch
```

Or use the package-level wrapper:

```bash
roslaunch neupan_ros lxw_car_simulation.launch
```

Publish a goal from RViz with `2D Nav Goal`, or start with a demo goal:

```bash
roslaunch neupan_ros example/lxw_car_simulation/launch/neupan_lxwcar_gazebo.launch publish_demo_goal:=true
```

The default launch enables the official-style dynamic obstacles. To run without `rvo_ros` while debugging the vehicle TF/control chain:

```bash
roslaunch neupan_ros lxw_car_simulation.launch dynamic_obstacles:=false
```

## Important dependency

The vehicle motion is driven by `scripts/ackermann_sim_driver.py`, not by a custom Gazebo Ackermann plugin. It subscribes to `/neupan_cmd_vel`, integrates the Ackermann model, publishes `/odom`, broadcasts `odom -> base_link`, publishes wheel/steering `/joint_states`, and moves the Gazebo model through `/gazebo/set_model_state`.

The dynamic obstacle setup follows `example/gazebo_limo`: it expects `rvo_ros` and the `libmulti_obs.so` Gazebo plugin to be available in the workspace/devel environment.
