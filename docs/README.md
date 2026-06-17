# Docs

This folder is for project notes that do not belong to a specific ROS package
or simulator implementation.

Current contents:

| File | Purpose |
| --- | --- |
| `wiki-guide.md` | Project wiki and setup notes. |

Keep machine-specific paths out of docs when possible. Prefer commands written
from the repository root, for example `python3 camera_calibration/test_camera.py`.

## Commands

List documentation files:

```bash
find docs -maxdepth 2 -type f | sort
```

Search project docs:

```bash
rg "aruco|camera|calibration|rosbag" docs README.md camera_calibration hardware simulators
```

Preview the wiki guide in a terminal:

```bash
sed -n '1,200p' docs/wiki-guide.md
```
