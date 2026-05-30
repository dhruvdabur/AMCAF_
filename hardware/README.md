# Hardware Workspace

This folder is the ROS 2 workspace for real vehicle control, CRSF messaging,
ArUco tracking, and hardware-facing tests.

## Build

From this folder:

```bash
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

Or from the repository root:

```bash
cd hardware
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

## Main Commands

Source the workspace before every `ros2 run` terminal:

```bash
source install/setup.bash
```

Start the CRSF bridge:

```bash
ros2 run crsf_ros2 crsf_ros
```

Run keyboard teleop:

```bash
ros2 run crsf_ros2 teleop_test
```

Run the ArUco track follower in preview-only dry-run mode:

```bash
ros2 run crsf_ros2 aruco_track_follower --dry-run --preview
```

Run the ArUco track follower with RC output enabled:

```bash
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview
```

Run with each controller variant:

```bash
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview --controller-mode pid
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview --controller-mode pid_velocity
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview --controller-mode pid_velocity_cbf
```

Load and save a PID tuning file:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --load-tuning \
  --tuning-file my_pid_tuning.json
```

Record a run with rosbag:

```bash
ros2 bag record /image_raw /drone/rc_command /mpc/ackermann_command
```

Run the simulator-to-actuator bridge in dry-run mode first:

```bash
ros2 run crsf_ros2 mpc_actuator --dry-run
```

List available ROS interfaces:

```bash
ros2 interface show rc_msgs/msg/RCMessage
ros2 interface show rc_msgs/msg/AckermannCommand
ros2 interface show rc_msgs/srv/CommandBool
```

## Safety

Any command using `--confirm-propulsion-safe` can move the vehicle. Keep wheels
off the ground during tuning, verify PWM ranges in dry-run mode first, and make
sure shutdown sends neutral commands before touching the vehicle.
