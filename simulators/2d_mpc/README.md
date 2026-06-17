# 2D MPC Simulator

This folder contains the fast 2D simulation workflow used for controller
development and benchmark metrics.

## Main Simulator

Show command options:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py --help
```

Run from the repository root:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py
```

Publish simulator control commands for the hardware bridge:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py --publish-control
```

## Controller Benchmarks

Run the available benchmark controllers:

```bash
python3 simulators/2d_mpc/pid_controller_benchmark.py --track oval
python3 simulators/2d_mpc/pid_velocity_controller_benchmark.py --track figure8
python3 simulators/2d_mpc/pid_velocity_cbf_controller_benchmark.py --track chicane
```

Compare saved benchmark metrics:

```bash
python3 simulators/2d_mpc/compare_controller_benchmarks.py
```

Save benchmark outputs into an explicit folder:

```bash
python3 simulators/2d_mpc/pid_controller_benchmark.py --track oval --output-dir benchmark_runs
python3 simulators/2d_mpc/pid_velocity_controller_benchmark.py --track oval --output-dir benchmark_runs
python3 simulators/2d_mpc/pid_velocity_cbf_controller_benchmark.py --track oval --output-dir benchmark_runs
```

Supported track names include `oval`, `figure8`, `chicane`, and `hairpin`.

## Hardware Bridge Pairing

Terminal 1, from `hardware/` after building and sourcing:

```bash
ros2 run crsf_ros2 crsf_ros
ros2 run crsf_ros2 mpc_actuator --dry-run
```

Terminal 2, from the repository root:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py --publish-control
```
