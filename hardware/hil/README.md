# hil ArUco Followers

Hardware-in-the-loop follower experiments live here. The active HIL path is now
the dynamic straight-road stack, with one reusable controller core, one dynamic
ROS node, and one ROS-free virtual simulation entrypoint.

## Package Layout

| Path | What it does |
| --- | --- |
| `straght_static.py` | Legacy straight-road ArUco follower entry point. |
| `straight_static.py` | Compatibility launcher for the older organized straight-road follower. |
| `ellipse_static.py` | Compatibility launcher for the older closed-ellipse follower. |
| `dynamic_straight.py` | Compatibility launcher for the active dynamic straight-road follower. |
| `simulation/` | ROS-free simulation harnesses and experiment assets. |
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
python3 -m crsf_ros2.hil.dynamic_straight \
  --dry-run \
  --preview
```

`--dry-run` keeps the node from publishing RC commands or arming anything, and
`--preview` opens the OpenCV preview window. Use module launch instead of running
`dynamic_straight.py` as a raw path because the HIL packages use relative imports.

## ROS 2 Launch

After changing `setup.py` or adding files, rebuild and source the hardware
workspace:

```bash
cd hardware
colcon build --packages-select crsf_ros2
source install/setup.bash
```

Then launch the active HIL follower:

```bash
ros2 run crsf_ros2 dynamic_straight \
  --dry-run \
  --preview \
  --no-pid-panel
```

The legacy misspelled executable `straght_static` is still present, and the
older `straight_static` and `ellipse_static` launchers still exist for
compatibility, but new runs should use `dynamic_straight`.

## Virtual Controller Test

Use the standalone virtual vehicle visualization when you want a drift-car-style
dynamic straight-road scene without camera input, ROS transport, ArUco
detection, or a controller loop:

```bash
python3 hardware/hil/simulation/virtual_vehicle.py \
  --duration-s 20 \
  --dt 0.05 \
  --preview
```

This virtual path builds a straight-road scene, advances a simple open-loop
kinematic vehicle, and optionally animates the result with the same
Matplotlib-style view used by the safe_control drift-car examples.

Useful virtual-test knobs are `--virtual-start-lateral-offset-px`,
`--virtual-start-heading-deg`, `--virtual-start-speed-pps`,
`--virtual-max-accel-pps2`, `--virtual-max-brake-pps2`, `--duration-s`,
`--dt`, `--dynamic-count`, and `--dynamic-speed-pps`.

If you want the older ROS-node-based synthetic pose path, it still exists in
`dynamic_straight_node.py` behind `--virtual-vehicle-test`:

```bash
ros2 run crsf_ros2 dynamic_straight \
  --dry-run \
  --preview \
  --no-pid-panel \
  --controller-mode pid_velocity_cbf_qp_ellipse \
  --virtual-vehicle-test
```

For a surprise-obstacle test where the controller does not receive a prior
obstacle map, use:

```bash
ros2 run crsf_ros2 straight_random_static_test
```

This launches the straight-road compatibility path with randomized hidden obstacles. They are drawn
gray while unknown and turn red once the virtual sensor reveals them to the
controller. Override the scenario with options such as
`--random-obstacle-count`, `--random-obstacle-seed`, and
`--random-obstacle-detection-range-px`.

## Static Obstacles

The straight-road HIL paths share the same laneless static-obstacle format. Add
virtual obstacles with JSON when you want to test avoidance behavior inside the
road:

```bash
ros2 run crsf_ros2 dynamic_straight \
  --dry-run \
  --preview \
  --controller-mode pid_velocity_cbf_qp_ellipse \
  --static-obstacles '[{"progress": 0.35, "offset": -0.45}, {"progress": 0.70, "offset": 0.45}]'
```

Each obstacle can set:

| Field | Meaning |
| --- | --- |
| `progress` | Position along the road from `0.0` at the left edge to `1.0` at the right edge. |
| `length_px` | Obstacle length along the road direction. |
| `width_px` | Obstacle width across the road. |
| `offset` | Lateral offset as a fraction of road half-width. |
| `lateral_offset_px` | Lateral offset in pixels. |

## Useful Options

| Option | Purpose |
| --- | --- |
| `--telemetry-prefix /aruco_track_follower/tuning` | Publishes live `std_msgs/Float64` tuning topics for PlotJuggler, including lateral error, heading error, roll, throttle, speed, and marker_seen. |
| `--no-telemetry` | Disables live tuning telemetry topics. |
| `--marker-id 0` | ArUco marker ID to track. |
| `--marker-dict DICT_4X4_50` | ArUco dictionary used by the marker. |
| `--aruco-parallax-factor 0.0` | Shifts the detected marker control point along the configured front edge by a fraction of marker size. Try small values such as `0.2` or `-0.2` when an angled camera makes the marker center look offset. |
| `--track-center-y 0.55` | Vertical placement of the straight road in the image. |
| `--road-length-x 0.82` | Fraction of image width used by the straight-road scene. |
| `--road-half-width-px 150` | Half-width of the virtual laneless road. |
| `--add-road-end-walls` | Adds static obstacle walls at the start and end of the straight road. |
| `--virtual-vehicle-test` | Runs the node-based straight-road controller against a synthetic vehicle instead of camera detections. |
| `--virtual-unlimited-path` | Scrolls and recycles the synthetic straight-road window for endless node-based virtual tests. |
| `--random-static-obstacles` | Spawns virtual obstacles that are hidden from the controller until detected. |
| `--traffic-scenario free_flow` | Selects the preset moving-obstacle scenario used by `dynamic_straight`. |
| `--dynamic-obstacle-count 6` | Number of moving obstacles for the dynamic straight scenarios. |
| `--controller-mode pid_velocity_cbf_qp_ellipse` | Enables car-centered elliptical QP-CBF filtering around lidar-detected obstacle corners. |
| `--cbf-h-px 42` | CBF obstacle barrier distance in pixels. Larger values keep more lidar clearance from obstacles and walls. Also available as `CBF h px` in the tuning panel. |
| `--cbf-alpha 1.0` | CBF aggressiveness. Lower values slow earlier; higher values allow more speed closer to limits. Also available as `CBF alpha x100` in the tuning panel. |
| `--metrics-file path.json` | Saves timestamped run metrics on exit. |

## Actuated Runs

Only run without `--dry-run` when the vehicle is physically safe and the CRSF
bridge is running:

```bash
ros2 run crsf_ros2 crsf_ros
ros2 run crsf_ros2 dynamic_straight \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf_qp_ellipse
```

The node refuses to send RC output unless `--confirm-propulsion-safe` is passed.

## PlotJuggler Tuning

Launch the follower with telemetry enabled, which is the default:

```bash
ros2 run crsf_ros2 dynamic_straight \
  --dry-run \
  --preview
```

Then open PlotJuggler and subscribe to:

```text
/aruco_track_follower/tuning/*
```

Useful plots while tuning are `lateral_error_px`, `heading_error_rad`,
`roll_pwm`, `throttle_pwm`, `track_speed_pps`, `marker_seen`, and
`safety_clearance_px`.
