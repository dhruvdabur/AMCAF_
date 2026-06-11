# crsf_ros2.submodules

Low-level protocol helpers bundled with the ROS package.

| File | Purpose |
| --- | --- |
| `crsf.py` | CRSF protocol encoding/decoding helpers used by the bridge. |

Code in this folder is intentionally isolated from HIL follower logic. Keep
protocol changes here and import them from the bridge layer.
