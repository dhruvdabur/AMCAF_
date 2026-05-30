# Hardware ROS 2 Packages

This folder is the `src/` directory of the hardware ROS 2 workspace.

| Package | Purpose |
| --- | --- |
| `rc_msgs/` | RC command and arming service interfaces. |
| `crsf_msgs/` | CRSF telemetry message interfaces. |
| `crsf_ros2/` | CRSF bridge, teleop tests, actuator bridge, and ArUco follower nodes. |

## Build

From the `hardware/` workspace:

```bash
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

Build one package while iterating:

```bash
colcon build --packages-select crsf_ros2
source install/setup.bash
```

List package executables:

```bash
ros2 pkg executables crsf_ros2
```

## CRSF Bridge

Run the CRSF ROS 2 node:

```bash
ros2 run crsf_ros2 crsf_ros
```

Watch RC command output:

```bash
ros2 topic echo /drone/rc_command
```

Check arming service availability:

```bash
ros2 service list | grep /drone/cmd/arming
```

## Steering-Only Check

The `steering_test` command sends steering on CRSF channel 1 while holding
channel 3 throttle neutral (`1500`), returning steering to center before
exiting.

Dry-run first:

```bash
ros2 run crsf_ros2 steering_test left --dry-run
```

With the CRSF adapter on `/dev/ttyUSB0`, request each position separately:

```bash
ros2 run crsf_ros2 steering_test left
ros2 run crsf_ros2 steering_test center
ros2 run crsf_ros2 steering_test right
```

Change serial hardware or servo endpoints when needed:

```bash
ros2 run crsf_ros2 steering_test left --port /dev/ttyUSB1 --left 1300 --duration 2
```

## Neutral-Centered Throttle Check

The `throttle_test` command sends throttle on CRSF channel 3 while holding
steering, pitch, and yaw centered.

Inspect generated commands without opening the serial port:

```bash
ros2 run crsf_ros2 throttle_test --dry-run
```

Only with the vehicle restrained, permit actual output:

```bash
ros2 run crsf_ros2 throttle_test --confirm-propulsion-safe --target-pwm 1510
```

## ArUco Track Follower

Start the camera publisher from the repository root:

```bash
python3 camera_calibration/test_camera.py --width 1920 --height 1080 --topic /image_raw
```

Dry-run the follower without RC output:

```bash
ros2 run crsf_ros2 aruco_track_follower --dry-run --preview
```

Run a one-lap PID + velocity + CBF test and save metrics:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf \
  --enable-lap-limit \
  --target-laps 1 \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

## Ackermann MPC Actuator Bridge

The `mpc_actuator` command accepts `/mpc/ackermann_command`
(`rc_msgs/msg/AckermannCommand`) and maps steering, route speed, and braking to
bounded `/drone/rc_command` PWM output.

Launch the simulator command publisher from the repository root:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py --publish-control
```

Inspect the default calibration without actuating:

```bash
ros2 run crsf_ros2 mpc_actuator --dry-run
```

Only with the driven wheels secured, enable physical output:

```bash
ros2 run crsf_ros2 mpc_actuator --confirm-propulsion-safe
```
