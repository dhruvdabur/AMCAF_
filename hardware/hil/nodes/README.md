# hil.nodes

ROS 2 node implementations for HIL executables.

| File | Purpose |
| --- | --- |
| `dynamic_straight_node.py` | Active straight-road HIL node with moving obstacles and dynamic-scenario logic. |
| `straight_static_node.py` | Older straight-road ArUco follower node kept for compatibility and shared simulation helpers. |
| `ellipse_static_node.py` | Older closed-ellipse follower node kept for compatibility. |

Files here should focus on ROS wiring and high-level behavior. Geometry,
configuration, drawing, metrics, and QoS helpers live in sibling packages.
