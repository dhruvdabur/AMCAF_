# PID Tuning Files

This note explains the two PID-related JSON files in this folder.

## Files

| File | Use |
| --- | --- |
| `PID_r1_ellipse.json` | Clean ArUco follower tuning file. Use this with `--tuning-file`. |
| `PID_r1_ellipse .json` | Metrics snapshot from a completed run. This filename has a space before `.json`; keep it as a record, but avoid using it as the main tuning file. |

## Load The Clean PID File

From the `hardware/` workspace after building and sourcing:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --load-tuning \
  --tuning-file PID_r1_ellipse.json
```

## Save Updated PID Values

While `aruco_track_follower` is running, tune the sliders and press `s` in the
PID tuning window. The node writes the current gains to the path passed with
`--tuning-file`.

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --tuning-file PID_r1_ellipse.json
```

## Run And Save Metrics Separately

Use a different file for metrics so tuned gains and run results do not get
mixed together:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --load-tuning \
  --tuning-file PID_r1_ellipse.json \
  --metrics-file aruco_track_follower_metrics.json
```

The metrics file is automatically timestamped by the follower, so each run gets
a separate JSON result.

## Current Clean Tuning Values

`PID_r1_ellipse.json` currently stores:

```text
controller_mode: pid_velocity
track_shape: oval
steering_kp_px: 1.4
steering_ki_px: 0.0
steering_kd_px: 0.25
heading_kp: 120.0
forward_pwm: 1590
min_forward_pwm: 1590
max_forward_pwm: 1600
target_track_speed_pps: 38.0
velocity_kp_pwm: 1.0
velocity_ki_pwm: 0.0
velocity_kd_pwm: 0.05
```
