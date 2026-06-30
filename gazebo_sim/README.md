# Gazebo Simulation

This folder contains the Gazebo simulation assets used for AMCaf vehicle-control experiments.
There are two simulation stacks here:

- `gazebo_worlds/` and `custom_worlds/`: the current Ignition/Gazebo workflow for the custom road, Prius model, ROS 2 bridge, and MPC controller.
- `car_demo/`: the older OSRF Prius demo built around ROS 1, Gazebo Classic, and Docker.

## Layout

| Path | Purpose |
| --- | --- |
| `gazebo_worlds/custom_road.sdf` | Main custom road world with road geometry, visual traffic actors, and the Prius include. |
| `gazebo_worlds/trajectory.csv` | Centerline waypoints used by the generated road, traffic actors, and MPC controller. |
| `gazebo_worlds/mpc_controller_node.py` | ROS 2 MPC controller node with OpenCV tuning controls. |
| `gazebo_worlds/run_mpc_control.sh` | Starts the ROS 2 to Gazebo bridge and the MPC controller node. |
| `gazebo_worlds/launch_all.sh` | Starts the custom world, bridge, and MPC controller together. |
| `gazebo_worlds/launch_custom.sh` | Starts `custom_road.sdf` in Ignition/Gazebo. |
| `custom_worlds/models/prius/` | Prius SDF model used by the current custom worlds. |
| `custom_worlds/models/ackermann_car/` | Simple Ignition Ackermann model with the built-in Ackermann steering system. |
| `custom_worlds/make_ellipse_world.py` | Generates the simpler segmented `ellipse_road.sdf`. |
| `custom_worlds/teleop_ackermann_ign.py` | Keyboard teleop publisher for a Gazebo model command topic. |
| `ackermann_description/urdf/ackermann_car.urdf` | URDF version of the simple Ackermann vehicle. |
| `car_demo/` | Legacy ROS 1/Gazebo Classic Prius demo. |

## Current MPC Workflow

From the repository root:

```bash
cd gazebo_sim/gazebo_worlds
./launch_all.sh
```

This script:

1. Starts Ignition/Gazebo with `custom_road.sdf`.
2. Starts `ros_gz_bridge` for:
   - `/model/prius/cmd_vel`
   - `/model/prius/odometry`
3. Starts `mpc_controller_node.py`.

The MPC node reads `trajectory.csv`, subscribes to Prius odometry, computes acceleration and steering with the shared HIL MPC controller, integrates acceleration into a velocity command, and publishes `geometry_msgs/msg/Twist` commands.

## Running Pieces Manually

Start only the custom road world:

```bash
cd gazebo_sim/gazebo_worlds
./launch_custom.sh
```

Start only the ROS 2 bridge and MPC node:

```bash
cd gazebo_sim/gazebo_worlds
./run_mpc_control.sh
```

Generate the simple ellipse world:

```bash
cd gazebo_sim/custom_worlds
python3 make_ellipse_world.py
```

Run keyboard teleop:

```bash
cd gazebo_sim/custom_worlds
python3 teleop_ackermann_ign.py
```

## Topics And Models

The current generated worlds include the `prius` model, so the control path uses:

- command topic: `/model/prius/cmd_vel`
- odometry topic: `/model/prius/odometry`

The simple `ackermann_car` model uses a different command path:

- command topic: `/model/ackermann_car/cmd_vel`
- odometry topic: `/model/ackermann_car/odometry`

If the car does not move, first confirm that the model included in the world matches the topic used by the teleop or controller script.

## Regenerating The Custom Road

`gazebo_worlds/trajectory.csv` is the source of truth for the generated path. The helper scripts use it to produce waypoint XML and to update road or actor trajectories:

```bash
cd gazebo_sim/gazebo_worlds
python3 generate_waypoints.py
python3 apply_waypoints_and_road.py
```

`apply_waypoints_and_road.py` rewrites `custom_road.sdf`, so inspect the diff before committing generated world changes.

## Dependencies

The current workflow expects:

- Ignition/Gazebo with the Ignition transport CLI available as `ign`.
- ROS 2, currently assumed by scripts to be Humble at `/opt/ros/humble/setup.bash`.
- `ros_gz_bridge`.
- Python packages used by the MPC node: `numpy`, `pandas`, `opencv-python`, `rclpy`, `casadi`, and `do_mpc`.

Some scripts still contain local absolute paths for this workspace. If moving the repository, update those paths or run from `/home/dhruv/amcaf`.
