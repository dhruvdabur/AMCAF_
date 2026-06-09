# control

Reusable, ROS-free controller logic for HIL scenarios.

`ellipse_static_controller.py` owns the closed-ellipse follower state machine, PID steering, QP-CBF filtering, virtual-lidar safety reporting, and metrics updates. ROS nodes and simulators should feed detections or simulator poses into `EllipseStaticController.process_detection(...)` and then publish the returned PWM commands through their own transport layer.
