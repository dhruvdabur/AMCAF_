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

More hardware commands are in `hardware/README.md`.
