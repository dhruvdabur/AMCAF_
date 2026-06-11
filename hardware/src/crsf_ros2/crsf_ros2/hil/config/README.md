# hil.config

Command-line configuration for HIL follower nodes.

| File | Purpose |
| --- | --- |
| `ellipse_static.py` | CLI arguments, defaults, validation, and startup config printing for the closed-ellipse follower. |
| `straight_static.py` | CLI arguments, defaults, validation, and startup config printing for the straight-road follower. |

Runtime nodes import configuration from here instead of defining defaults
inline. That keeps hardware limits, controller defaults, and tuning-file
settings in one place.
