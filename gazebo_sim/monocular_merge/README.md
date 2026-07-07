# Monocular Merge

This folder is a small ArUco-marker version of the `charucal` idea: use shared
fiducials to merge two camera views, then express detections in one metric
world frame.

For your setup:

- camera 0 is expected to see ArUco IDs `1,2,3,4`
- camera 1 is expected to see ArUco IDs `3,4,5,6`
- IDs `3,4` are the overlap used to estimate camera-1 pose in camera-0 space
- marker ID `1` is the origin, so marker `1` is `(0, 0, 0)`
- marker ID `0` is the target pose to report
- every marker side length is `0.20 m`

The output pose `id0 in id1` is the transform from marker 0 into marker 1's
coordinate frame. Its translation is the center of marker 0 measured from the
center of marker 1, in metres.

## Run

Use separate intrinsics for the two cameras when you have them:

```bash
python3 -m monocular_merge.aruco_merge \
  --cam0 /dev/video0 \
  --cam1 /dev/video4 \
  --intrinsics0 camera_calibration/camera_intrinsics.yaml \
  --intrinsics1 camera_calibration/camera_intrinsics.yaml \
  --marker-size-m 0.20 \
  --dictionary DICT_4X4_50 \
  --fourcc MJPG \
  --frame-width 640 \
  --frame-height 360 \
  --preview
```

For one JSON result:

```bash
python3 -m monocular_merge.aruco_merge \
  --cam0 /dev/video0 \
  --cam1 /dev/video4 \
  --intrinsics0 camera_calibration/camera_intrinsics.yaml \
  --intrinsics1 camera_calibration/camera_intrinsics.yaml \
  --fourcc MJPG \
  --frame-width 640 \
  --frame-height 360 \
  --once \
  --json \
  --output-json monocular_merge/latest_pose.json
```

You can also pass image paths instead of camera indices for an offline check:

```bash
python3 -m monocular_merge.aruco_merge \
  --cam0 path/to/camera0.png \
  --cam1 path/to/camera1.png \
  --once
```

## How The Merge Works

OpenCV estimates each marker as `T_camera_marker`, using the calibrated camera
matrix, distortion coefficients, and the `0.20 m` square marker model.

For each shared marker, the camera-to-camera transform is:

```text
T_camera0_camera1 = T_camera0_shared_marker * inverse(T_camera1_shared_marker)
```

The tool averages the estimates from shared IDs `3` and `4`, transforms camera-1
detections into camera-0 coordinates, then computes:

```text
T_id1_id0 = inverse(T_camera0_id1) * T_camera0_id0
```

That final transform is the pose of marker `0` relative to marker `1`.

## Important Assumption

This does not need surveyed positions for markers `2`-`6`; it builds the
relative marker layout live from the cameras. If you want a fixed floor-map
coordinate system that remains valid when marker `1` is not visible, you still
need to measure and provide the physical poses of the anchor markers.
