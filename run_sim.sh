#!/bin/bash
# Helper script to launch the follow-the-gap standalone simulation

export PYTHONPATH=/home/dhruv/.local/lib/python3.10/site-packages:$PYTHONPATH

python3 -m simulation.run_simulation \
  --dry-run \
  --preview \
  --controller-mode pid_velocity_cbf_qp_ellipse \
  --virtual-vehicle-test \
  --virtual-unlimited-path \
  --static-obstacles '[{"progress": 0.35, "offset": -0.45}, {"progress": 0.70, "offset": 0.45}]' \
  --load-tuning \
  --tuning-file aruco_track_follower_tuning.json
