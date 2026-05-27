<h1>Implementation of CRSF protocol on ROS2 Humble</h1>

CRSF is a telemetry protocol that can be used for both RC control and to get telemetry information from the vehicle/flight controller on a compatible RC transmitter.

<h2>Prerequisites</h2>

- Ubuntu 22.04 LTS
- ROS2 Humble

<h2>Installation Instructions</h2>

```
mkdir ~/crsf_ws
cd crsf_ws
git clone https://github.com/arunser/crsf-ros2.git src
```
```
cd ~/crsf_ws
colcon build
source install/setup.bash
```

To run the CRSF ROS2 node, use the following command;

```
ros2 run crsf_ros2 crsf_ros
```

<h2>Steering-only check</h2>

The `steering_test` command sends steering on CRSF channel 1 while holding
channel 3 throttle neutral (`1500`), returning steering to center before
exiting.

This command must be connected to a CRSF transmitter module or other device
that accepts handset-side channel commands. An ExpressLRS receiver sends
channel commands out to a flight controller; it does not accept steering
commands from this script. The test uses handset-to-transmitter-module CRSF
frames at `400000` baud.

Before testing, keep the vehicle clear of the ground and disconnect drive
power where possible. Confirm packet values without accessing hardware:

```
ros2 run crsf_ros2 steering_test left --dry-run
```

With the CRSF adapter on `/dev/ttyUSB0`, request each position separately:

```
ros2 run crsf_ros2 steering_test left
ros2 run crsf_ros2 steering_test center
ros2 run crsf_ros2 steering_test right
```

Each command holds the requested position for one second and then centers the
steering. Change serial hardware or servo endpoints when needed:

```
ros2 run crsf_ros2 steering_test left --port /dev/ttyUSB1 --left 1300 --duration 2
```

For higher transmitter-module rates, pass the configured handset baud rate,
for example `--baudrate 921600`. Receiver-to-flight-controller CRSF is a
different connection whose ExpressLRS UART default is `420000` baud.

<h2>Neutral-centered throttle check</h2>

The `throttle_test` command sends throttle on CRSF channel 3 while
holding steering, pitch, and yaw centered. For a centered ESC, neutral
throttle is `1500`; the command ramps from neutral to a small default target
(`1510`), holds briefly, and restores neutral before exiting. Use
`--target-pwm` from `1400` to `1600` to check either direction carefully.

Inspect generated commands without opening the serial port:

```
ros2 run crsf_ros2 throttle_test --dry-run
```

This command can spin propulsion. Remove propellers or keep driven wheels off
the ground, secure the vehicle, and do not run `crsf_ros` simultaneously. To
permit actual output on `/dev/ttyUSB0`:

```
ros2 run crsf_ros2 throttle_test --confirm-propulsion-safe --target-pwm 1510
```

Change ramp timing or the serial connection when needed:

```
ros2 run crsf_ros2 throttle_test --confirm-propulsion-safe --target-pwm 1510 --ramp-duration 5 --hold-duration 0.2 --port /dev/ttyUSB1
```
