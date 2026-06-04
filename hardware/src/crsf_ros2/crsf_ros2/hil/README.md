# hil ArUco Followers

Hardware-in-the-loop follower experiments live here. These nodes use the same
ROS 2 camera and RC command interfaces as the main `crsf_ros2` follower, but
are kept separate so hil-specific track layouts can change without disturbing
the general ArUco follower.

## Files

| File | What it does |
| --- | --- |
| `straght_static.py` | ArUco marker follower for a plain straight landscape road. It draws two straight lane centerlines over `/image_raw`, tracks marker ID `0`, and can optionally avoid JSON-defined static obstacles. By default it runs on a clear road with no obstacles. |

`straght_static.py` is the renamed hil copy of the earlier
`hil/aruco_track_follower.py`.

## Camera Publisher

Start the camera publisher from the repository root before launching a hil
follower:

```bash
python3 camera_calibration/test_camera.py \
  --skip-v4l2-ctl \
  --device 2 \
  --topic /image_raw \
  --output camera_calibration/camera_testing.yaml
```

This opens the Lenovo camera on `/dev/video2`, previews the feed, and publishes
ROS 2 images on `/image_raw`.

## Direct Python Launch

Direct Python is useful while editing because it does not require rebuilding the
ROS package:

```bash
python3 hardware/src/crsf_ros2/crsf_ros2/hil/straght_static.py \
  --dry-run \
  --preview
```

`--dry-run` keeps the node from publishing RC commands or arming anything.
`--preview` opens the OpenCV debug window with the straight-road overlay. The
PID/CBF tuning panel opens by default; pass `--no-pid-panel` only when you want
to hide it.

## ROS 2 Launch

After changing `setup.py` or adding files, rebuild and source the hardware
workspace:

```bash
cd hardware
colcon build --packages-select crsf_ros2
source install/setup.bash
```

Then launch the hil straight-road follower:

```bash
ros2 run crsf_ros2 straght_static \
  --dry-run \
  --preview \
  --no-pid-panel
```

## Static Obstacles

The straight-road hil follower always treats the start and end of the road as
walls. Add more virtual static obstacles with JSON when you want to test
avoidance behavior inside the road:

```bash
ros2 run crsf_ros2 straght_static \
  --dry-run \
  --preview \
  --controller-mode pid_velocity_cbf \
  --static-obstacles '[{"lane": 0, "progress": 0.45}, {"lane": 1, "progress": 0.70}]'
```

Each obstacle can set:

| Field | Meaning |
| --- | --- |
| `lane` | Lane index, `0` or `1`. |
| `progress` | Position along the road from `0.0` at the left edge to `1.0` at the right edge. |
| `length_px` | Obstacle length along the road direction. |
| `width_px` | Obstacle width across the lane. |

## Useful Options

| Option | Purpose |
| --- | --- |
| `--marker-id 0` | ArUco marker ID to track. |
| `--marker-dict DICT_4X4_50` | ArUco dictionary used by the marker. |
| `--track-center-y 0.55` | Vertical placement of the straight road in the image. |
| `--road-length-y 0.82` | Fraction of image width used by the straight road. The name is inherited from the S-road code. |
| `--road-lane-width-px 150` | Lane spacing and road boundary width in pixels. |
| `--controller-mode pid_velocity_cbf` | Enables velocity control and safety slowdown around lane/error/obstacle limits. |
| `--cbf-h-px 42` | CBF obstacle barrier distance in pixels. Larger values keep more lidar clearance from obstacles and walls. Also available as `CBF h px` in the tuning panel. |
| `--cbf-alpha 1.0` | CBF aggressiveness. Lower values slow earlier; higher values allow more speed closer to limits. Also available as `CBF alpha x100` in the tuning panel. |
| `--metrics-file path.json` | Saves timestamped run metrics on exit. |

## Actuated Runs

Only run without `--dry-run` when the vehicle is physically safe and the CRSF
bridge is running:

```bash
ros2 run crsf_ros2 crsf_ros
ros2 run crsf_ros2 straght_static \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf
```

The node refuses to send RC output unless `--confirm-propulsion-safe` is passed.
