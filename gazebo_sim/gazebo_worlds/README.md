# Gazebo Worlds & Controllers Workspace

This folder contains the main custom road world asset, reference trajectory path coordinates, launch scripts, and ROS 2 controller nodes for AMCaf vehicle-control experiments.

## Controller Nodes

### 1. MPC Controller Node (`mpc_controller_node.py`)
A closed-loop path-tracking and obstacle-avoidance node using Model Predictive Control (MPC) paired with Control Barrier Functions (CBF).
* **Parity & Architecture**: Imports its core optimization logic from the shared HIL control stack (`from hardware.hil.controllers.mpc_cbf import MPCController`).
* **Vehicle-Frame Control**: Transforms the world-frame trajectory coordinates from `trajectory.csv` into the local vehicle/body frame at each iteration, solving the CasADi/do-mpc optimization relative to the vehicle's origin ($x_0 = [0, 0, 0, v]^T$).
* **Real-World Actuation**: Estimates real car velocity from tracked coordinate differences, scales computed steering and throttle commands, and publishes `RCMessage` outputs on `/drone/rc_command` alongside simulation commands on `/model/prius/cmd_vel`.
* **Live Tuning**: Spawns an interactive OpenCV tuning panel for real-time target speed, prediction horizon, and state cost weight tuning, saving parameters to `mpc_tuning.json` upon pressing the `S` key.

### 2. PID Controller Node (`pid_controller_node.py`)
A drop-in replacement for the MPC node using a traditional proportional-integral-derivative path follower.
* **Core Logic**: Imports its control math from `hardware/hil/controllers/pid.py`.
* **Shared Interface**: Subscribes to the same `/pika/pose` topic, estimates velocity identically, and maps steering/throttle commands to matching RCMessage outputs.
* **Tuning Panel**: Spawns a parallel OpenCV tuning UI for adjusting PID gains ($K_p$, $K_i$, $K_d$) and look-ahead target parameters on the fly.

---

## Launch & Execution Scripts

### 1. `launch_custom.sh`
Launches the custom road simulation world (`custom_road.sdf`) in Ignition/Gazebo.

```bash
./launch_custom.sh
```

### 2. `run_mpc_control.sh`
Launches the ROS 2 to Ignition/Gazebo bridge (bridging `/clock`, `/model/prius/cmd_vel`, `/model/prius/odometry`, `/world/custom_road_world/set_pose`, and `/lidar2D/scan`) and runs the `mpc_controller_node.py`.

```bash
./run_mpc_control.sh
```

### 3. `run_pid_control.sh`
Launches the same ROS 2 bridge and runs `pid_controller_node.py`.

```bash
./run_pid_control.sh
```

### 4. `launch_all.sh`
A convenience script that starts the custom world, configures the ROS 2 bridge, and runs `mpc_controller_node.py` simultaneously.

---

## Teleoperation & Utility Scripts

### 1. `teleop_real_car.py`
A keyboard teleoperation node used to manually drive the real car or test servo/actuator range limits (Throttle Pitch and Steering Roll). Publishes command messages directly to `/drone/rc_command`.

```bash
python3 teleop_real_car.py --confirm-propulsion-safe
```

### 2. `plot_lateral_error.py`
Reads the telemetry logged by the controllers in `mpc_telemetry.csv`, computes overall tracking metrics (RMSE, max cross-track error), and saves a three-panel performance analysis plot to `mpc_performance_plot.png`.

```bash
python3 plot_lateral_error.py
```

### 3. `toggle_obstacles.py`
A utility script to quickly toggle visual obstacles on the track.

---

## Reference Trajectories & Road Assets

* **`trajectory.csv`**: Source of truth containing centerline waypoints `(x, y, yaw)` mapped in the Pika Sense world coordinate system.
* **`custom_road.sdf`**: Main world file describing the track geometry.
* **`apply_waypoints_and_road.py` / `generate_waypoints.py`**: Scripts used to regenerate path waypoints and update road geometry layout coordinates.
