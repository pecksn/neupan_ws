# lxw_car_simulation

Gazebo simulation entry for the LXW Ackermann car with NeuPAN.

## What is included

- `urdf/`: Ackermann URDF/Xacro with the real LXW car CAD exterior and simplified collision geometry.
- `meshes/lxwcar_cad.stl`: binary visual mesh exported from `src/models/lxwcar.step`.
- `launch/neupan_lxwcar_gazebo.launch`: starts Gazebo, spawns the car, starts `robot_state_publisher`, starts `neupan_node.py`, and optionally starts RViz/demo goal.
- `worlds/lxw_car_obstacles.world`: default local static-obstacle world.
- `worlds/gazebo_world_complex_20.world`: optional complex world from `example/gazebo_limo`.
- `models/`: reusable fallback box and cylinder obstacle models.
- `config/neupan_config_sim.yaml`: NeuPAN simulation config aligned with the LXW car geometry.
- `config/rvo_config_complex.yaml`: RVO dynamic obstacle config copied from the official Gazebo LIMO example.
- `rviz/lxw_car_simulation.rviz`: RViz view for `/scan`, TF, robot model, and NeuPAN markers/paths.
- `scripts/publish_demo_goal.py`: optional latched goal publisher for quick smoke tests.

## Sensor And Avoidance Notes

The simulation keeps the real MID360 mounting frame at `livox_frame = 0.1 0 0.25` relative to `base_link`, matching `example/lxw_car/launch/full_system_run.launch`. Because Gazebo's `ray` sensor is only a 2D slice while the real car uses MID360 point cloud plus `pointcloud_to_laserscan`, the simulated `laser_link` is placed higher as a virtual scan slice. The obstacle worlds add taller collision cylinders so that this slice is actually visible in `/scan`.

The CAD file uses millimetres. The exported STL bounds are about 0.938 × 0.745 × 0.359 m, so the URDF scales it by 0.001 and offsets it to centre the car and put its lowest point on the ground. The mesh is visual only; the existing joints, wheel collisions, and laser/IMU links remain active. The planner footprint and scan self-filter width are 0.75 m to cover the CAD exterior. To show the original primitive visuals for debugging, launch with `cad_visual:=false`.

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

The default launch uses local static obstacles and does not need `rvo_ros`. To use the complex dynamic world, first install/build `rvo_ros`, `libmulti_obs.so`, and the world's referenced models, then run:

```bash
roslaunch neupan_ros lxw_car_simulation.launch dynamic_obstacles:=true world_name:=$(rospack find neupan_ros)/example/lxw_car_simulation/worlds/gazebo_world_complex_20.world
```

## Important dependency

The vehicle motion is driven by `scripts/ackermann_sim_driver.py`, not by a custom Gazebo Ackermann plugin. It subscribes to `/neupan_cmd_vel`, integrates the Ackermann model, publishes `/odom`, broadcasts `odom -> base_footprint` (with the URDF providing `base_footprint -> base_link`), publishes wheel/steering `/joint_states`, and moves the Gazebo model through `/gazebo/set_model_state`. Commands expire after 0.5 seconds by default (`cmd_timeout` launch argument); this stops the car if the controller exits or its messages stop.

The dynamic obstacle setup follows `example/gazebo_limo`: it expects `rvo_ros` and the `libmulti_obs.so` Gazebo plugin to be available in the workspace/devel environment.
