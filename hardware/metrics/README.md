# ArUco Track Follower Metrics

This folder stores JSON metrics produced by `aruco_track_follower` runs.

Each run should use the same base output path:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

The follower automatically appends a timestamp, so the saved files look like:

```text
metrics/aruco_track_follower_metrics_YYYYMMDD_HHMMSS.json
```

Keep tuning files such as `PID_r1_ellipse.json` separate from metrics files.
Tuning files are loaded with `--tuning-file`; metrics files are run results
saved with `--metrics-file`.
