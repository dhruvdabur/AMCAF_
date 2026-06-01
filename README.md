# dev-hardware

This branch is intentionally scoped to hardware-facing work. It contains only
the camera calibration utilities and the ROS 2 hardware workspace used for the
vehicle.

## Folder Overview

| Folder | Purpose |
| --- | --- |
| `camera_calibration/` | USB camera preview, `/image_raw` publishing, chessboard calibration, and manual intrinsic tuning tools. |
| `hardware/` | ROS 2 workspace for CRSF control, RC messages, teleop tests, ArUco track following, tuning files, and run metrics. |

## Camera Calibration

Run the camera publisher:

```bash
python3 camera_calibration/test_camera.py --width 1920 --height 1080 --topic /image_raw
```

Run chessboard calibration:

```bash
python3 camera_calibration/camera_chessboard_calibrator.py --columns 8 --rows 6 --square-size 0.025
```

Open the manual intrinsic tuner:

```bash
python3 camera_calibration/camera_intrinsics_tuner.py
```

More camera commands are in `camera_calibration/README.md`.

## Hardware Workspace

Build and source the ROS 2 workspace:

```bash
cd hardware
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

Run manual teleop:

```bash
ros2 run crsf_ros2 teleop_test
```

Run the ArUco follower in dry-run preview mode:

```bash
ros2 run crsf_ros2 aruco_track_follower --dry-run --preview
```

Run the ArUco follower with RC output only after the vehicle is restrained:

```bash
ros2 run crsf_ros2 aruco_track_follower --confirm-propulsion-safe --preview
```

## ArUco Track Follower Launch Commands

Start the camera publisher first from the repository root:

```bash
python3 camera_calibration/test_camera.py --width 1920 --height 1080 --topic /image_raw
```

Then build and source the hardware workspace:

```bash
cd hardware
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

Dry-run the follower without publishing RC commands:

```bash
ros2 run crsf_ros2 aruco_track_follower --dry-run --preview
```

Run PID-only control:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid
```

Run PID plus velocity control:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity
```

Run PID plus velocity control plus CBF:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf
```

Load saved PID values:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --load-tuning \
  --tuning-file PID_r1_ellipse.json
```

Save updated PID values while running:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --tuning-file PID_r1_ellipse.json
```

Tune the sliders and press `s` in the PID window to write the file.

Run until a target lap count is completed:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --enable-lap-limit \
  --target-laps 1
```

Run with metrics saved to a timestamped file:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

Run a full tuned PID + velocity + CBF experiment:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf \
  --track-shape oval \
  --load-tuning \
  --tuning-file PID_r1_ellipse.json \
  --enable-lap-limit \
  --target-laps 1 \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

Record the run with rosbag:

```bash
ros2 bag record /image_raw /drone/rc_command
```

Useful checks while it is running:

```bash
ros2 topic hz /image_raw
ros2 topic echo /drone/rc_command
ros2 interface show rc_msgs/msg/RCMessage
```

More hardware commands are in `hardware/README.md`.
