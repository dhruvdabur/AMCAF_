# Camera Calibration

Camera utilities live here so the repository root stays focused on project
entry points and saved run artifacts.

## Scripts

| Script | Purpose |
| --- | --- |
| `test_camera.py` | Opens the USB camera, previews the live feed, and publishes frames to ROS 2 on `/image_raw`. |
| `camera_chessboard_calibrator.py` | Detects chessboard corners and estimates camera intrinsics automatically. |
| `camera_intrinsics_tuner.py` | Manual OpenCV intrinsic/distortion tuning helper. |
| `camera_extrinsics_tuner.py` | Manual camera pose/extrinsic tuning helper with projected grid overlay. |

## Common Commands

Show script options:

```bash
python3 camera_calibration/test_camera.py --help
python3 camera_calibration/camera_chessboard_calibrator.py --help
python3 camera_calibration/camera_intrinsics_tuner.py --help
python3 camera_calibration/camera_extrinsics_tuner.py --help
```

Run the live camera publisher:

```bash
python3 camera_calibration/test_camera.py
```

Publish 1080p frames to `/image_raw`:

```bash
python3 camera_calibration/test_camera.py --width 1920 --height 1080 --topic /image_raw
```

Publish 720p frames:

```bash
python3 camera_calibration/test_camera.py --width 1280 --height 720 --topic /image_raw
```

Open the optional brightness/exposure slider panel:

```bash
python3 camera_calibration/test_camera.py --controls
```

Run automatic chessboard calibration:

```bash
python3 camera_calibration/camera_chessboard_calibrator.py --columns 8 --rows 6 --square-size 0.025
```

Run ROS 2 Humble camera calibration against the `/image_raw` publisher:

```bash
sudo apt install ros-humble-camera-calibration
python3 camera_calibration/test_camera.py --width 1920 --height 1080 --topic /image_raw
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.025 --ros-args -r image:=/image_raw
```

Or use the helper with the same defaults:

```bash
bash camera_calibration/run_ros_cameracalibrator.sh
```

Accept chessboard samples manually with SPACE:

```bash
python3 camera_calibration/camera_chessboard_calibrator.py --manual --samples 25 --output camera_calibration/camera_intrinsics.yaml
```

Open the manual intrinsic tuner and save with `s`:

```bash
python3 camera_calibration/camera_intrinsics_tuner.py --load camera_calibration/camera_intrinsics.yaml --output camera_calibration/tuned_intrinsics.yaml
```

The intrinsic tuner also opens image sliders for hardware camera controls when
the device exposes them, plus preview-only brightness, contrast, gamma,
saturation, hue shift, sharpness, blur, and CLAHE controls. Press `a` to reset
the preview image sliders without touching the intrinsic values.

Open the manual extrinsic tuner and save with `s`:

```bash
python3 camera_calibration/camera_extrinsics_tuner.py --intrinsics camera_calibration/tuned_intrinsics.yaml --output camera_calibration/camera_extrinsics.yaml
```

The extrinsic tuner projects a world-plane grid into the live image. Adjust
roll, pitch, yaw, and translation until the overlay matches the scene, or press
`p`/SPACE while a chessboard is visible to snap the pose with `solvePnP`.
Saved output includes both `world_to_camera` and `camera_to_world` transforms.

The chessboard size is the number of inner corners, not the number of printed
squares. For example, a board with 9 by 7 printed squares has 8 by 6 inner
corners.

## Outputs

Camera property snapshots and calibration YAML files can be saved anywhere, but
prefer paths under this folder or a run-specific output folder so they remain
portable:

```bash
python3 camera_calibration/test_camera.py --output camera_calibration/camera_testing.yaml
```

## ROS 2 Checks

After starting `test_camera.py`, confirm frames are being published:

```bash
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /image_raw --once
```
