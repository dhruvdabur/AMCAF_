# hil Controllers

Controller definitions shared by hil follower nodes live here.

| File | Purpose |
| --- | --- |
| `modes.py` | Names the supported controller modes: `pid`, `pid_velocity`, and `pid_velocity_cbf`. |
| `pid.py` | Defines `PIDController`, `PIDVelocityCBFController`, and the image-space `VirtualLidar`. |

Current controller behavior is still orchestrated by each follower node. For
`straght_static.py`, the main mode branch is in `throttle_to_pwm()`, while this
folder holds the reusable pieces that future hil follower files can share.

`PIDVelocityCBFController` follows the simulator pattern from
`simulators/2d_mpc/controller_benchmark_common.py`: velocity PID produces a
nominal forward command, then the CBF safety filter scales that command toward
neutral as clearance shrinks.

`VirtualLidar` follows the shape of the `av_control_guide` omni-directional
lidar: it samples obstacle polygon contours, bins returns by angle around the
car/marker center, and keeps the nearest return per angular bin. In the preview,
those lidar returns are drawn as magenta rays from the marker center to obstacle
hits.
