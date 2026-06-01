# crsf_msgs

ROS 2 interface package for CRSF-related telemetry messages.

## Messages

| Message | Purpose |
| --- | --- |
| `Attitude.msg` | Vehicle attitude fields from CRSF telemetry. |
| `BatterySensor.msg` | Battery voltage/current/capacity telemetry. |
| `FlightMode.msg` | Flight mode state text. |

Build this package from the hardware workspace:

```bash
cd hardware
colcon build --packages-select crsf_msgs
source install/setup.bash
```

Show interface definitions:

```bash
ros2 interface show crsf_msgs/msg/Attitude
ros2 interface show crsf_msgs/msg/BatterySensor
ros2 interface show crsf_msgs/msg/FlightMode
```

List package interfaces:

```bash
ros2 interface package crsf_msgs
```
