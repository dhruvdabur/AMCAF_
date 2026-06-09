# hil.metrics

Run metrics for HIL followers.

| File | Purpose |
| --- | --- |
| `follower.py` | Tracks CTE, heading error, speed error, effort, safety clearance, CBF interventions, marker loss, and lap count. |

Metrics are kept separate from controller code so experiments can change their
logging without touching command generation.
