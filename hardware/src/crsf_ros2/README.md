# crsf_ros2

ROS 2 Python package for CRSF serial output, manual tests, simulator actuator
bridging, and ArUco marker track following.

## Nodes

| Command | Purpose |
| --- | --- |
| `crsf_ros` | CRSF serial bridge. |
| `teleop_test` | Manual keyboard control path used as the known-good reference. |
| `aruco_track_follower` | Camera-frame virtual track follower using an ArUco marker on the vehicle. |
| `mpc_actuator` | Maps simulator Ackermann commands to bounded RC PWM output. |
| `steering_test` | Steering endpoint check. |
| `throttle_test` | Neutral-centered throttle check. |
| `throttle_topic_test` | Topic-based throttle test helper. |

## ArUco Follower Example

In one terminal, publish camera frames from the repository root:

```bash
python3 camera_calibration/test_camera.py
```

In another terminal, build and source the hardware workspace:

```bash
cd hardware
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

Then run the follower:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

Press `s` in the tuning window to save PID settings. Press `q` to stop, send
neutral commands, save metrics, and exit.

## Launch Commands

Build after editing this package:

```bash
cd hardware
colcon build --packages-select crsf_ros2
source install/setup.bash
```

Run manual teleop:

```bash
ros2 run crsf_ros2 teleop_test
```

Run CRSF bridge:

```bash
ros2 run crsf_ros2 crsf_ros
```

Run steering and throttle tests safely:

```bash
ros2 run crsf_ros2 steering_test left --dry-run
ros2 run crsf_ros2 throttle_test --dry-run
ros2 run crsf_ros2 throttle_test --confirm-propulsion-safe --target-pwm 1510
```

Run ArUco follower without actuating:

```bash
ros2 run crsf_ros2 aruco_track_follower --dry-run --preview
```

Run controller variants:

```bash
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview --controller-mode pid
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview --controller-mode pid_velocity
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview --controller-mode pid_velocity_cbf
```

Run a target-lap experiment with tuning and metrics files:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf \
  --track-shape oval \
  --enable-lap-limit \
  --target-laps 1 \
  --load-tuning \
  --tuning-file my_pid_tuning.json \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

Record the important topics during a run:

```bash
ros2 bag record /image_raw /drone/rc_command
```

Inspect topics and message types:

```bash
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /drone/rc_command
ros2 interface show rc_msgs/msg/RCMessage
```
