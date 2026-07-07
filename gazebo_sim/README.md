# Gazebo Simulation

This folder contains the Gazebo simulation assets used for AMCaf vehicle-control experiments.
There are two simulation stacks here:

- `gazebo_worlds/` and `custom_worlds/`: the current Ignition/Gazebo workflow for the custom road, Prius model, ROS 2 bridge, and MPC controller.
- `car_demo/`: the older OSRF Prius demo built around ROS 1, Gazebo Classic, and Docker.

## How to Launch the HIL Closed-Loop Control System

To launch the real-world closed-loop Hardware-in-the-Loop (HIL) experiment, run each component in separate terminals:

### Step 1: Launch Pika Sense Localization (Mocap/Camera tracking)
Launch the Pika Sense node to track and publish the real car's pose on `/pika/pose`:
```bash
ros2 run pika_sense_ros sdk_bridge_node --ros-args -p serial_port:=/dev/ttyUSB0
```

### Step 2: Launch Monocular Camera Merger (Alternative Localization)
If using camera-based ArUco marker merging, launch the monocular merge script to fuse views from two cameras (e.g., `/dev/video0` and `/dev/video4`) and estimate coordinates relative to the origin marker:
```bash
python3 -m monocular_merge.aruco_merge \
  --cam0 /dev/video0 \
  --cam1 /dev/video4 \
  --intrinsics0 camera_calibration/camera_intrinsics.yaml \
  --intrinsics1 camera_calibration/camera_intrinsics.yaml \
  --marker-size-m 0.20 \
  --dictionary DICT_4X4_50 \
  --fourcc MJPG \
  --frame-width 640 \
  --frame-height 360 \
  --preview
```

### Step 3: Launch CRSF Bridge Serial Driver
Start the CRSF bridge node to parse incoming `/drone/rc_command` inputs and send PWM actuation commands to the physical ELRS transmitter (dynamically falls back to connected USB serial ports):
```bash
ros2 run crsf_ros2 crsf_ros
```

### Step 4: Start the Gazebo Digital Twin Simulation
Launch the virtual custom road world in Gazebo/Ignition:
```bash
cd gazebo_sim/gazebo_worlds
./launch_custom.sh
```

### Step 5: Start the Path-Tracking Controller Node
Choose one of the tracking controller nodes to compute steering and throttle outputs and run closed-loop control:
* **MPC Controller Node**:
  ```bash
  cd gazebo_sim/gazebo_worlds
  ./run_mpc_control.sh
  ```
* **PID Controller Node**:
  ```bash
  cd gazebo_sim/gazebo_worlds
  ./run_pid_control.sh
  ```

> [!NOTE]
> Both `mpc_controller_node.py` and `pid_controller_node.py` import their core path-tracking controller logic from the shared HIL control path (`hardware/hil/controllers/` or `from hardware.hil.controllers`).

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

## Live MPC Tuning & Saving

The MPC controller node launches an interactive **OpenCV Tuning Panel** alongside the simulation, allowing you to tune the controller weights and observe telemetry in real time:

* **Tuning Sliders**: Adjust target speed, horizon steps ($T$), and cost weights ($w_{xy}$, $w_{yaw}$, $w_{steer}$, $w_{vel}$) dynamically.
* **Live Update**: The CasADi solver re-optimizes its weights instantly on slider adjustment without losing tracking waypoint context.
* **Live Graph**: A rolling cross-track lateral error plot displays at the bottom of the panel with a green centerline to track accuracy.
* **Saving Weights**: Press the **`S` key** on the OpenCV panel to save your current parameter tuning to [`gazebo_sim/gazebo_worlds/mpc_tuning.json`](file:///home/dhruv/amcaf/gazebo_sim/gazebo_worlds/mpc_tuning.json). These values load automatically on subsequent runs.

## Performance Analysis & Plotting

During execution, the controller logs state and control variables to [`gazebo_sim/gazebo_worlds/mpc_telemetry.csv`](file:///home/dhruv/amcaf/gazebo_sim/gazebo_worlds/mpc_telemetry.csv).

To analyze tracking quality post-run, generate performance plots by running:
```bash
cd gazebo_sim/gazebo_worlds
./plot_lateral_error.py
```
This computes tracking metrics (RMSE, Max Error) and saves a three-panel performance analysis plot to [`gazebo_sim/gazebo_worlds/mpc_performance_plot.png`](file:///home/dhruv/amcaf/gazebo_sim/gazebo_worlds/mpc_performance_plot.png) showing:
1. **Signed Lateral Error** over time.
2. **Actual vs. Target Speed** over time.
3. **Steering Commands** over time.

## Simulation Physics & Performance Optimizations

1. **Auto-Unpause**: [`launch_all.sh`](file:///home/dhruv/amcaf/gazebo_sim/gazebo_worlds/launch_all.sh) features an unpause service loop that automatically unpauses Gazebo physics, getting the Prius driving immediately without manual clicks.
2. **Sensor Rendering Disabled**: LEGACY cameras, sonars, and Lidars are commented out of the Prius [`model.sdf`](file:///home/dhruv/amcaf/gazebo_sim/custom_worlds/models/prius/model.sdf), eliminating massive GPU/CPU rendering overhead.
3. **Seamless Ground Plane**: Road segment collision boxes are disabled (visual-only), and the infinite ground plane collider is raised to $z = 0.02$ to match the road surface. This prevents tire-seam micro-collisions and vehicle jittering.
4. **ROS Sim Time**: The MPC node is synchronized with Gazebo's `/clock` (`use_sim_time=True`), preventing command lag and control loop synchronization jitter.

## Hardware-in-the-Loop (HIL) & Real Car Control

The control scripts can operate in a Hardware-in-the-Loop (HIL) closed-loop mode to actuate the physical car based on real-time tracking:

### 1. Control Loop Structure
* **Vehicle Localization**: Subscribes to `/pika/pose` (geometry_msgs/msg/PoseStamped) coming from the Pika Sense tracking system.
* **Velocity Estimation**: Estimates the real car's speed by computing the finite difference of the filtered positions over time ($dt = 0.05$s) using a low-pass filter to smooth out sensor noise.
* **Vehicle-Frame Trajectory Transformation**: To avoid world coordinate drift and origin offsets, reference trajectory waypoints from `trajectory.csv` are transformed into the vehicle's local/body frame at every control iteration. The MPC solver resolves tracking errors relative to this moving local frame (starting from $x_{0\_local} = [0, 0, 0, v]^T$).
* **RC Actuation**: The computed steering and throttle values are mapped and published as `rc_roll` (steering, scaled between 1300–1700) and `rc_pitch` (throttle, scaled between 1582–1590 when driving) on the `/drone/rc_command` topic (RCMessage).
* **Digital Twin Synchronization**: The simulation Prius is teleported to match the real car's tracking coordinates in Gazebo, enabling the simulator's front 2D LiDAR (`/lidar2D/scan`) to act as a virtual sensor to detect obstacles relative to the road.

### 2. How to Launch
To run the closed-loop HIL controllers with the physical vehicle:

1. **Launch the CRSF Serial Driver Node**:
   This node connects to the ELRS transmitter via serial and publishes `/drone/rc_command` messages to the car. It automatically detects if the transmitter is connected on `/dev/ttyUSB0` or falls back dynamically to `/dev/ttyUSB1`, `/dev/ttyUSB2`, `/dev/ttyACM0`, or `/dev/ttyACM1`.
   ```bash
   ros2 run crsf_ros2 crsf_ros
   ```

2. **Launch the Controller Node**:
   * For the **MPC Controller**:
     ```bash
     cd gazebo_sim/gazebo_worlds
     ./run_mpc_control.sh
     ```
   * For the **PID Controller**:
     ```bash
     cd gazebo_sim/gazebo_worlds
     ./run_pid_control.sh
     ```

3. **Restrained Teleop & Safety Diagnostics**:
   To manually override or test actuator range limits:
   ```bash
   python3 gazebo_sim/gazebo_worlds/teleop_real_car.py --confirm-propulsion-safe
   ```

## Dependencies

The current workflow expects:
* Ignition/Gazebo with the Ignition transport CLI available as `ign`.
* ROS 2 (Humble/Iron) sourced.
* `ros_gz_bridge`.
* Python packages: `numpy`, `pandas`, `opencv-python`, `matplotlib`, `rclpy`, `casadi`, and `do_mpc`.

> [!NOTE]
> Legacy ROS 1 packages (`prius_msgs`, `prius_description`, `car_demo`, `ackermann_description`) are skipped automatically during `colcon build` via `COLCON_IGNORE` markers to ensure clean compilation.

