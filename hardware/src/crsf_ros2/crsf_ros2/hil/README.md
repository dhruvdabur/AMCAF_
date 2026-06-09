# hil ArUco Followers

Hardware-in-the-loop follower experiments live here. These nodes use the same
ROS 2 camera and RC command interfaces as the main `crsf_ros2` follower, but
are kept separate so hil-specific track layouts can change without disturbing
the general ArUco follower.

## Package Layout

| Path | What it does |
| --- | --- |
| `straght_static.py` | Legacy straight-road ArUco follower entry point. |
| `ellipse_static.py` | Compatibility launcher for the closed-ellipse follower. The implementation is split across the packages below. |
| `nodes/` | ROS 2 node classes and executable `main()` functions. |
| `config/` | CLI arguments, defaults, validation, and startup config printing. |
| `road/` | Image-space road geometry, obstacle polygons, free-space intervals, and clearances. |
| `metrics/` | Per-run tracking, safety, lap, and effort metrics. |
| `ui/` | OpenCV preview drawing helpers. |
| `runtime/` | ROS runtime helpers such as QoS profiles. |
| `common/` | Small shared math helpers. |
| `controllers/` | PID, velocity CBF, and QP-CBF controller primitives. |

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

Module launch is useful while editing because it can run from source without
installing console-script wrappers:

```bash
cd hardware/src/crsf_ros2
python3 -m crsf_ros2.hil.ellipse_static \
  --dry-run \
  --preview
```

`--dry-run` keeps the node from publishing RC commands or arming anything, and
`--preview` opens the OpenCV debug window. Use module launch instead of running
`ellipse_static.py` as a raw path because the HIL packages use relative imports.

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

Launch the ellipse-road follower the same way:

```bash
ros2 run crsf_ros2 ellipse_static \
  --dry-run \
  --preview \
  --no-pid-panel
```

The ROS executable name is unchanged even though the implementation now lives in
`hil/nodes/ellipse_static_node.py`.

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
| `--track-radius-x 0.20` | Horizontal ellipse radius as a fraction of image width for `ellipse_static.py`. |
| `--track-radius-y 0.24` | Vertical ellipse radius as a fraction of image height for `ellipse_static.py`. |
| `--road-length-y 0.82` | Fraction of image width used by the straight road. The name is inherited from the S-road code. |
| `--road-lane-width-px 150` | Lane spacing for straight-road modes; road half-width for `ellipse_static.py`. |
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
