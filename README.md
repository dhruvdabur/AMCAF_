# dev-hardware

This branch is scoped to hardware-facing work for the AMCAF vehicle stack. It
contains the camera calibration tools, the ROS 2 hardware workspace, CRSF/RC
interfaces, actuator test nodes, ArUco follower nodes, and HIL controller
experiments.

Use this branch when you are working with the physical camera, CRSF adapter,
RC PWM messages, ArUco marker tracking, PID/velocity/CBF tuning, or hardware
in the loop road experiments.

## Branch

Check the active branch and its remote tracking branch:

```bash
git branch --show-current
git status -sb
git rev-parse --abbrev-ref --symbolic-full-name @{u}
```

Expected branch:

```text
dev-hardware
```

Expected upstream:

```text
origin/dev-hardware
```

Switch to this branch:

```bash
git fetch origin
git switch dev-hardware
```

Create a new branch from it:

```bash
git switch -c my-hardware-change
```

## File Structure

Generated folders such as `build/`, `install/`, `log/`, `__pycache__/`,
`build_isolated/`, and `devel_isolated/` may appear locally after running ROS
or Python tools. They are not the main source files.

```text
.
├── README.md
├── camera_calibration/
│   ├── README.md
│   ├── camera_chessboard_calibrator.py
│   ├── camera_extrinsics_tuner.py
│   ├── camera_intrinsics_tuner.py
│   ├── run_ros_cameracalibrator.sh
│   ├── test_camera.py
│   ├── camera_intrinsics.yaml
│   ├── camera_testing.yaml
│   └── ros_camera_info.yaml
├── gazebo_sim/
│   ├── README.md
│   └── gazebo_worlds/
│       ├── custom_road.sdf
│       ├── trajectory.csv
│       ├── mpc_controller_node.py
│       ├── pid_controller_node.py
│       ├── run_mpc_control.sh
│       └── run_pid_control.sh
└── hardware/
    ├── README.md
    ├── metrics/
    │   └── README.md
    └── src/
        ├── README.md
        ├── rc_msgs/
        │   ├── CMakeLists.txt
        │   ├── README.md
        │   ├── package.xml
        │   ├── msg/
        │   │   ├── AckermannCommand.msg
        │   │   └── RCMessage.msg
        │   └── srv/
        │       └── CommandBool.srv
        ├── crsf_msgs/
        │   ├── CMakeLists.txt
        │   ├── README.md
        │   ├── package.xml
        │   └── msg/
        │       ├── Attitude.msg
        │       ├── BatterySensor.msg
        │       └── FlightMode.msg
        └── crsf_ros2/
            ├── README.md
            ├── package.xml
            ├── setup.cfg
            ├── setup.py
            ├── resource/
            │   └── crsf_ros2
            ├── test/
            │   ├── test_copyright.py
            │   ├── test_derivative_cbf.py
            │   ├── test_flake8.py
            │   └── test_pep257.py
            └── crsf_ros2/
                ├── __init__.py
                ├── aruco_track_follower.py
                ├── mpc_actuator.py
                ├── ros2_crsf.py
                ├── steering_test.py
                ├── teleop_test.py
                ├── throttle_test.py
                ├── throttle_topic_test.py
                ├── submodules/
                │   ├── __init__.py
                │   └── crsf.py
                └── hil/
                    ├── README.md
                    ├── __init__.py
                    ├── straght_static.py
                    └── controllers/
                        ├── README.md
                        ├── __init__.py
                        ├── modes.py
                        └── pid.py
```

## Package Map

| Path | Purpose |
| --- | --- |
| `camera_calibration/` | USB camera preview, ROS `/image_raw` publishing, chessboard calibration, intrinsic tuning, and extrinsic tuning tools. |
| `gazebo_sim/` | Gazebo simulator assets, world definitions, digital twin bridge scripts, and path-tracking controller nodes. |
| `gazebo_sim/gazebo_worlds/` | Custom road simulation world, reference path waypoints, and the ROS 2 MPC and PID controller nodes with OpenCV tuning. |
| `hardware/` | ROS 2 workspace for physical vehicle control and hardware-facing experiments. |
| `hardware/src/rc_msgs/` | RC command message and arming service interfaces. |
| `hardware/src/crsf_msgs/` | CRSF telemetry message interfaces. |
| `hardware/src/crsf_ros2/` | Python ROS 2 package with CRSF bridge, tests, ArUco follower, MPC actuator bridge, and HIL nodes. |
| `hardware/src/crsf_ros2/crsf_ros2/hil/` | Hardware-in-the-loop road follower experiments. |
| `hardware/src/crsf_ros2/crsf_ros2/hil/controllers/` | PID, velocity PID, virtual lidar, and derivative-based CBF controller primitives. |

## ROS 2 Build

Build all hardware packages:

```bash
cd hardware
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

Build only the Python control package while iterating:

```bash
cd hardware
colcon build --packages-select crsf_ros2
source install/setup.bash
```

Source the workspace in every terminal that runs `ros2` commands:

```bash
cd hardware
source install/setup.bash
```

List package executables:

```bash
ros2 pkg executables crsf_ros2
```

## Camera Launch Commands

Start the camera publisher from the repository root:

```bash
python3 camera_calibration/test_camera.py \
  --width 1920 \
  --height 1080 \
  --topic /image_raw
```

Start the Lenovo camera on `/dev/video2`, skip `v4l2-ctl`, preview frames, and
save camera settings:

```bash
python3 camera_calibration/test_camera.py \
  --skip-v4l2-ctl \
  --device 2 \
  --topic /image_raw \
  --output camera_calibration/camera_testing.yaml
```

Run chessboard calibration:

```bash
python3 camera_calibration/camera_chessboard_calibrator.py \
  --columns 8 \
  --rows 6 \
  --square-size 0.025
```

Run ROS 2 Humble camera calibration from `/image_raw`:

```bash
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator \
  --size 6x8 \
  --square 0.025 \
  --ros-args -r image:=/image_raw
```

Open the manual intrinsic tuner:

```bash
python3 camera_calibration/camera_intrinsics_tuner.py
```

Open the manual extrinsic tuner after saving intrinsics:

```bash
python3 camera_calibration/camera_extrinsics_tuner.py \
  --intrinsics camera_calibration/tuned_intrinsics.yaml
```

## CRSF And RC Launch Commands

Start the CRSF serial bridge:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 crsf_ros
```

Run keyboard teleop:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 teleop_test
```

Run steering safely in dry-run mode:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 steering_test left --dry-run
```

Run steering against the adapter:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 steering_test left
ros2 run crsf_ros2 steering_test center
ros2 run crsf_ros2 steering_test right
```

Run throttle safely in dry-run mode:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 throttle_test --dry-run
```

Run a restrained low-throttle check:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 throttle_test \
  --confirm-propulsion-safe \
  --target-pwm 1510
```

Inspect RC messages:

```bash
ros2 topic echo /drone/rc_command
ros2 interface show rc_msgs/msg/RCMessage
ros2 interface show rc_msgs/msg/AckermannCommand
ros2 interface show rc_msgs/srv/CommandBool
```

## ArUco Follower Launch Commands

Start the camera publisher first from the repository root:

```bash
python3 camera_calibration/test_camera.py \
  --width 1920 \
  --height 1080 \
  --topic /image_raw
```

In another terminal, build and source the hardware workspace:

```bash
cd hardware
colcon build --packages-select rc_msgs crsf_msgs crsf_ros2
source install/setup.bash
```

Dry-run the follower without publishing RC commands:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --dry-run \
  --preview
```

Run PID-only control:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid
```

Run PID plus velocity control:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity
```

Run PID plus velocity control plus CBF:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf
```

Run the two-lane S-curve road with virtual static obstacles:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --dry-run \
  --preview \
  --track-shape s_curve_road \
  --controller-mode pid_velocity_cbf
```

Override the default virtual obstacles with JSON:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --dry-run \
  --preview \
  --track-shape s_curve_road \
  --static-obstacles '[{"lane": 0, "progress": 0.25}, {"lane": 1, "progress": 0.55}]'
```

Load saved PID values:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --load-tuning \
  --tuning-file PID_r1_ellipse.json
```

Save updated PID values while running:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --tuning-file PID_r1_ellipse.json
```

Tune the sliders and press `s` in the PID window to write the file. Press `q`
to stop, send neutral commands, save metrics, and exit.

Run until a target lap count is completed:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --enable-lap-limit \
  --target-laps 1
```

Run with metrics saved to a file:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

Run a full tuned PID + velocity + CBF experiment:

```bash
ros2 run crsf_ros2 aruco_track_follower \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf \
  --track-shape oval \
  --load-tuning \
  --tuning-file PID_r1_ellipse.json \
  --enable-lap-limit \
  --target-laps 1 \
  --metrics-file metrics/aruco_track_follower_metrics.json
```

## HIL Straight-Road Launch Commands

Direct Python launch is useful while editing because it does not require a ROS
package rebuild:

```bash
python3 hardware/src/crsf_ros2/crsf_ros2/hil/straght_static.py \
  --dry-run \
  --preview
```

Run the HIL straight-road follower through ROS 2:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 straght_static \
  --dry-run \
  --preview \
  --no-pid-panel
```

Run the straight-road follower with PID + velocity + CBF:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 straght_static \
  --dry-run \
  --preview \
  --controller-mode pid_velocity_cbf
```

Add virtual static obstacles:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 straght_static \
  --dry-run \
  --preview \
  --controller-mode pid_velocity_cbf \
  --static-obstacles '[{"lane": 0, "progress": 0.45}, {"lane": 1, "progress": 0.70}]'
```

Run the HIL follower with RC output only after the vehicle is restrained:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 straght_static \
  --confirm-propulsion-safe \
  --preview \
  --controller-mode pid_velocity_cbf
```

## HIL Dynamic Obstacles & Scenarios

Run the straight-road follower with dynamic obstacles and presets:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 ftg_dynamic_straight \
  --dry-run \
  --preview \
  --traffic-scenario endless_walls \
  --virtual-vehicle-test \
  --virtual-unlimited-path \
  --controller-mode pid_velocity_cbf_qp_ellipse \
  --load tuning \
  --tuning-file /home/dhruv/amcaf/aruco_track_follower_tuning1.json \
  --executor-threads 8
```

Available Preset Traffic Scenarios:
- `endless_walls`: Stationary blocking walls and dynamic slow-moving traffic on an endless path.
- `head_on`: Ego vehicle approaches a single obstacle head-on (side boundary walls disabled).
- `three_sparse`, `looping_flow`, `free_flow`, etc.

## Telemetry Recording & Analysis

All core metrics (errors, safety clearance barrier $h$, control effort, actuator saturations, and pose accuracy values) are published dynamically to ROS 2 topics under `/dynamic_straight/tuning/` or `/aruco_track_follower/tuning/`.

Record a run with `rosbag`:
```bash
ros2 bag record -a
```

### Telemetry Decoding and Analysis Scripts

1. **Decompress Bag File**:
   If the bag uses `zstd` compression:
   ```bash
   zstd -d bags/<bag_dir>/<file_name>.db3.zstd -o bags/<bag_dir>/<file_name>.db3
   ```

2. **Decode Bag Payload to CSV**:
   Extract all Float64 CDR topics to raw and pivoted aligned CSV spreadsheets:
   ```bash
   python3 bags/decode_bag.py bags/<bag_folder_name>
   ```

3. **Plot Telemetry Analysis**:
   Generate the CBF-QP analysis plot comparing danger limits, actual actuator commands, and obstacle boundary clearance $h$:
   ```bash
   python3 bags/graph_generator.py bags/<bag_folder_name>/pivoted_messages.csv
   ```
   *Plots are saved automatically to `cbf_qp_analysis.png` in the bag folder.*

4. **Calculate Detailed Performance Metrics Report**:
   Compile a performance report outlining lateral error (CTE), heading tracking, pose accuracy, control efforts, safety violation ratios, and FTG planner solve latency:
   ```bash
   python3 bags/telemetry_analyser.py bags/<bag_folder_name>
   ```
   *Reports are saved automatically to `telemetry_analysis_report.md` in the bag folder.*

## MPC Actuator Bridge Launch Commands

Launch a simulator command publisher from the repository root when available:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py --publish-control
```

Inspect actuator mapping without sending physical output:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 mpc_actuator --dry-run
```

Enable physical output only after the driven wheels are secured:

```bash
cd hardware
source install/setup.bash
ros2 run crsf_ros2 mpc_actuator --confirm-propulsion-safe
```

## Recording And Debugging

Record the important topics during a run:

```bash
cd hardware
source install/setup.bash
ros2 bag record /image_raw /drone/rc_command /mpc/ackermann_command
```

Check camera frame rate:

```bash
ros2 topic hz /image_raw
```

Inspect RC output:

```bash
ros2 topic echo /drone/rc_command
```

List available topics, services, and interfaces:

```bash
ros2 topic list
ros2 service list
ros2 interface package rc_msgs
ros2 interface package crsf_msgs
ros2 pkg executables crsf_ros2
```

## Tests

Compile-check the edited Python files:

```bash
python3 -m py_compile \
  hardware/src/crsf_ros2/crsf_ros2/hil/controllers/pid.py \
  hardware/src/crsf_ros2/crsf_ros2/hil/straght_static.py
```

Run the derivative CBF tests when `pytest` is installed:

```bash
PYTHONPATH=hardware/src/crsf_ros2 pytest -q \
  hardware/src/crsf_ros2/test/test_derivative_cbf.py
```

Run package tests through colcon:

```bash
cd hardware
colcon test --packages-select crsf_ros2
colcon test-result --verbose
```

## Safety Notes

Commands that include `--confirm-propulsion-safe` can move the vehicle. Before
using them, restrain the vehicle, keep driven wheels off the ground during
tuning, verify PWM values in `--dry-run` mode, and confirm the node returns
neutral commands on shutdown.

Prefer this order for physical runs:

```text
1. Camera publisher
2. ROS workspace sourced
3. Dry-run follower or actuator test
4. CRSF bridge
5. Restrained actuated run with --confirm-propulsion-safe
```

More specific docs live in:

```text
gazebo_sim/README.md
camera_calibration/README.md
hardware/README.md
hardware/src/README.md
hardware/src/crsf_ros2/README.md
hardware/src/crsf_ros2/crsf_ros2/hil/README.md
hardware/src/crsf_ros2/crsf_ros2/hil/controllers/README.md
```

## Gazebo Digital Twin & HIL Real Car Control

The system supports running closed-loop control on the physical vehicle while simulating obstacles in Gazebo (HIL digital twin mode):

### 1. Launching simulation twin
To start the Gazebo simulation world:
```bash
cd gazebo_sim/gazebo_worlds
./launch_custom.sh
```

### 2. Launching controllers (with Pika Sense tracking feedback)
To run the MPC or PID controller nodes (which read `/pika/pose` to estimate real car velocity, align reference waypoints to the vehicle's local frame, and output `rc_roll` and `rc_pitch` commands to `/drone/rc_command`):

* **MPC Controller**:
  ```bash
  cd gazebo_sim/gazebo_worlds
  ./run_mpc_control.sh
  ```

* **PID Controller**:
  ```bash
  cd gazebo_sim/gazebo_worlds
  ./run_pid_control.sh
  ```

* **Real Car Teleoperation**:
  ```bash
  python3 gazebo_sim/gazebo_worlds/teleop_real_car.py --confirm-propulsion-safe
  ```
