# hil.config

Command-line configuration for HIL follower nodes.

| File | Purpose |
| --- | --- |
| `dynamic_straight.py` | CLI arguments, defaults, validation, and startup config printing for the active dynamic straight-road follower. |
| `straight_static.py` | Older straight-road CLI configuration kept for compatibility. |
| `ellipse_static.py` | Older closed-ellipse CLI configuration kept for compatibility. |

Runtime nodes import configuration from here instead of defining defaults
inline. That keeps hardware limits, controller defaults, and tuning-file
settings in one place.
