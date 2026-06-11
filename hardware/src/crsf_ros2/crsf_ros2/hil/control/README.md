# control

Reusable, ROS-free controller logic for HIL scenarios.

`ellipse_static_controller.py` owns the shared follower state machine, PID steering, QP-CBF filtering, virtual-lidar safety reporting, and metrics updates. `straight_static_controller.py` reuses that core with open-road straight geometry. ROS nodes and simulators should feed detections or simulator poses into the controller `process_detection(...)` method and then publish the returned PWM commands through their own transport layer.
