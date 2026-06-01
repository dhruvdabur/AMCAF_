# rc_msgs

ROS 2 interface package for RC command and actuator bridge messages.

## Interfaces

| Interface | Purpose |
| --- | --- |
| `msg/RCMessage.msg` | Raw RC channel PWM command message. |
| `msg/AckermannCommand.msg` | Steering/speed/brake command used by the simulator bridge. |
| `srv/CommandBool.srv` | Boolean command service used for arming/disarming. |

Build this package from the hardware workspace:

```bash
cd hardware
colcon build --packages-select rc_msgs
source install/setup.bash
```

Show interface definitions:

```bash
ros2 interface show rc_msgs/msg/RCMessage
ros2 interface show rc_msgs/msg/AckermannCommand
ros2 interface show rc_msgs/srv/CommandBool
```

List package interfaces:

```bash
ros2 interface package rc_msgs
```
