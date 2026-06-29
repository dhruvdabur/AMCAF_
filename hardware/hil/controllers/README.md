# hil Controllers

Controller definitions shared by hil follower nodes live here.

| File | Purpose |
| --- | --- |
| `cbf_qp_ellipse.py` | Defines the car-centered elliptical QP-CBF safety filter for acceleration and steering commands. |
| `modes.py` | Names the supported controller modes: `pid`, `pid_velocity`, `pid_velocity_cbf`, and `pid_velocity_cbf_qp_ellipse`. |
| `pid.py` | Defines `PIDController`, `PIDVelocityCBFController`, and the image-space `VirtualLidar`. |

Current controller behavior is primarily orchestrated by
`hil/nodes/dynamic_straight_node.py`, while this folder holds the reusable
pieces shared across the HIL stack.

`PIDVelocityCBFController` follows the simulator pattern from
`simulators/2d_mpc/controller_benchmark_common.py`: velocity PID produces a
nominal forward command, then the CBF safety filter scales that command toward
neutral as clearance shrinks.

`VirtualLidar` follows the shape of the `av_control_guide` omni-directional
lidar: it samples obstacle polygon contours, bins returns by angle around the
car/marker center, and keeps the nearest return per angular bin. In the preview,
those lidar returns are drawn as magenta rays from the marker center to obstacle
hits.
