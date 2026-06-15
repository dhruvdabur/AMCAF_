# hil.runtime

ROS runtime helpers shared by HIL nodes.

| File | Purpose |
| --- | --- |
| `qos.py` | Sensor QoS profile tuned for low-latency image processing. |

Runtime helpers are kept small so node files can read like orchestration code
instead of ROS boilerplate.
