# hil.road

Image-space road geometry and obstacle helpers.

| File | Purpose |
| --- | --- |
| `ellipse.py` | Builds the closed laneless elliptical road, static obstacle polygons, tangent/normal vectors, free lateral intervals, and obstacle clearances. |

This package has no ROS dependencies; it operates on numbers and NumPy arrays
so it is easy to test separately from hardware.
