# lxw_car_simulation

Gazebo simulation entry for the LXW Ackermann car with NeuPAN.

## What is included

- `urdf/`: primitive-shape Ackermann URDF/Xacro for Gazebo.
- `launch/neupan_lxwcar_gazebo.launch`: starts Gazebo, spawns the car, starts `robot_state_publisher`, starts `neupan_node.py`, and optionally starts RViz/demo goal.
- `worlds/lxw_car_obstacles.world`: static obstacle world.
- `models/`: reusable box and cylinder obstacle models.
- `config/neupan_config_sim.yaml`: NeuPAN simulation config aligned with the LXW car geometry.
- `rviz/lxw_car_simulation.rviz`: RViz view for `/scan`, TF, robot model, and NeuPAN markers/paths.
- `scripts/publish_demo_goal.py`: optional latched goal publisher for quick smoke tests.

## Run

```bash
cd /home/nuc/lxw/neupan_ws
catkin_make
source devel/setup.bash
roslaunch neupan_ros example/lxw_car_simulation/launch/neupan_lxwcar_gazebo.launch
```

Publish a goal from RViz with `2D Nav Goal`, or start with a demo goal:

```bash
roslaunch neupan_ros example/lxw_car_simulation/launch/neupan_lxwcar_gazebo.launch publish_demo_goal:=true
```

## Important dependency

The URDF currently uses `libgazebo_ros_ackerman.so`, matching the existing Ackermann xacro style in this workspace. If Gazebo reports that this plugin cannot be loaded, install or build the Ackermann Gazebo plugin first, or replace it with another Gazebo drive plugin plus a steering-angle-to-yaw-rate bridge.
