# crsf_ros2 Python Package

This package contains the ROS 2 Python nodes for the CRSF hardware bridge,
manual actuator tests, and camera-based HIL followers.

## Layout

| Path | Purpose |
| --- | --- |
| `ros2_crsf.py` | CRSF serial bridge that receives RC messages and writes CRSF frames. |
| `teleop_test.py`, `steering_test.py`, `throttle_test.py` | Manual hardware test entry points. |
| `mpc_actuator.py` | MPC command to actuator command bridge. |
| `aruco_track_follower.py` | General ArUco track follower. |
| `hil/` | Hardware-in-the-loop follower experiments and reusable helpers. |
| `submodules/` | Low-level protocol helpers vendored with the package. |

Keep imports inside this package package-relative when code crosses local
subpackages. Console scripts in `setup.py` should remain small entry points
into these modules.
