# tuning

OpenCV field-tuning panels and JSON tuning-file helpers.

`straight_static_tuning.py` is the active tuning helper used by the dynamic
straight path. `ellipse_static_tuning.py` remains in the tree for the older
ellipse compatibility path. Both keep slider UI and tuning persistence separate
from reusable controllers and from ROS transport code.
