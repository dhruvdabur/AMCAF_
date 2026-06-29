# hil.road

Image-space road geometry and obstacle helpers.

| File | Purpose |
| --- | --- |
| `straight.py` | Builds the active open laneless straight road, safe-control-inspired scene, and optional end-wall obstacles. |
| `ellipse.py` | Older closed-road geometry plus shared obstacle, tangent/normal, and clearance helpers. |

This package has no ROS dependencies; it operates on numbers and NumPy arrays
so it is easy to test separately from hardware.
