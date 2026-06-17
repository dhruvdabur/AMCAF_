# Source Libraries

This folder holds reusable source code and upstream/reference material used by
the simulator entry points.

| Folder | Purpose |
| --- | --- |
| `av_control_guide/` | Vehicle models, controllers, planners, localization, mapping, and educational reference simulations. |

Project entry points live outside this folder in `simulators/` and `hardware/`.
Keep new runnable scripts there unless they are reusable library code.

## Commands

Run the upstream/reference test suite:

```bash
cd src/av_control_guide
python3 -m pytest test
```

Run a specific reference simulation:

```bash
python3 src/av_control_guide/src/simulations/path_tracking/mpc_path_tracking/mpc_path_tracking.py
```

Search reusable controller code:

```bash
rg "class .*Controller|def update" src/av_control_guide/src/components/control
```
