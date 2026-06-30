# Legacy Prius Car Demo

This directory contains the OSRF Prius demo adapted into the AMCaf Gazebo tree.
It is separate from the current `gazebo_worlds/` ROS 2 MPC workflow.

Use this directory when you specifically need the older ROS 1/Gazebo Classic Prius demo, joystick translator, RViz configuration, or `PriusHybridPlugin` implementation.

## What Is Here

| Path | Purpose |
| --- | --- |
| `car_demo/` | Catkin package containing the Gazebo Classic world, launch file, joystick node, and Prius model plugin. |
| `prius_description/` | Prius URDF and mesh assets. |
| `prius_msgs/` | ROS 1 `Control.msg` used to command throttle, brake, steering, and gear state. |
| `build_demo.bash` | Builds the Docker image. |
| `run_demo.bash` | Runs the demo container through `rocker`. |

## Runtime Stack

This demo targets:

- ROS Kinetic.
- Gazebo 9 / Gazebo Classic.
- Catkin.
- Docker plus `rocker`.

The launch file is:

```bash
car_demo/car_demo/launch/demo.launch
```

It starts:

- `gazebo_ros` with `car_demo/worlds/mcity.world`.
- `robot_state_publisher`.
- `fake_localization`.
- joystick nodes for `/dev/input/js0` and `/dev/input/js1`.
- `joystick_translator`.
- RViz with `car_demo/rviz/demo.rviz`.
- a spawned Prius URDF model.

## Controls

The primary ROS message is `prius_msgs/Control`:

- `throttle`: `0.0` to `1.0`
- `brake`: `0.0` to `1.0`
- `steer`: `-1.0` to `1.0`
- `shift_gears`: `NO_COMMAND`, `NEUTRAL`, `FORWARD`, or `REVERSE`

The `PriusHybridPlugin` also exposes Ignition transport handlers for command and utility topics, including `/cmd_vel`, `/cmd_gear`, `/cmd_mode`, `/prius/reset`, and `/prius/stop`.

## Build

From this directory:

```bash
./build_demo.bash
```

This builds the local Docker image:

```text
osrf/car_demo
```

## Run

Connect a game controller if you want joystick input, then run:

```bash
./run_demo.bash
```

RViz opens with the car and sensor output, and Gazebo opens with the simulation world. You can drive with a controller or click into the Gazebo window and use keyboard controls.

For a Logitech F710 in XInput mode:

- Right stick controls throttle and brake.
- Left stick controls steering.
- `Y` shifts to drive.
- `A` shifts to reverse.
- `B` shifts to neutral.

## Relationship To The Current Gazebo Workflow

The current AMCaf Gazebo workflow lives one level up in `../gazebo_worlds/` and `../custom_worlds/`.
That path uses Ignition/Gazebo, ROS 2, `ros_gz_bridge`, and `mpc_controller_node.py`.

Use `car_demo/` only when working on the legacy ROS 1 Prius demo or the original OSRF plugin stack.
