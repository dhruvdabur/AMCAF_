# Simulators

Simulator entry points are grouped here. Shared controller, vehicle, obstacle,
and visualization code is kept in `src/av_control_guide/src/components/`.

| Folder | Purpose |
| --- | --- |
| `2d_mpc/` | Fast 2D controller development, path tracking, benchmarking, and plots. |

Run commands from the repository root so relative imports and output paths stay
predictable.

## Commands

Install Python dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

Run the 2D simulator:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py
```

Run the 2D simulator as a hardware command publisher:

```bash
python3 simulators/2d_mpc/mpc_path_tracking.py --publish-control
```

Run controller benchmarks:

```bash
python3 simulators/2d_mpc/pid_controller_benchmark.py --track oval
python3 simulators/2d_mpc/pid_velocity_controller_benchmark.py --track figure8
python3 simulators/2d_mpc/pid_velocity_cbf_controller_benchmark.py --track chicane
```
