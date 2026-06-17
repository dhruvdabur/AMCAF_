# Mentor Demo Benchmark

| Scenario | Controller | Collisions | Min clearance px | CBF active | Min h | Mean QP ms | Max QP ms | Max slack | Steer smooth |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| free_flow | pid | 35 | -174.8 | 0.0% | n/a | 0.00 | 0.00 | 0.000 | 23316.0 |
| free_flow | pid_velocity_cbf_qp_ellipse | 7 | -24.8 | 97.5% | 14.480 | 14.46 | 59.89 | 0.000 | 1053.0 |

Use the PID rows as baseline and the CBF-QP rows to show safety intervention, barrier value, solve time, and smoothness tradeoffs.
