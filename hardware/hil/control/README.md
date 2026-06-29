# control

Reusable, ROS-free controller logic for HIL scenarios.

`dynamic_straight_controller.py` is the active controller entrypoint. It sits on
top of `straight_static_controller.py`, which in turn reuses the shared logic in
`ellipse_static_controller.py` for follower state, PID steering, QP-CBF
filtering, virtual-lidar safety reporting, and metrics updates. The dynamic
layer adds explicit static, detected-random, and moving-obstacle obstacle
groups so ROS nodes and simulators can feed poses into `process_detection(...)`
and publish the resulting PWM commands through their own transport layer.
