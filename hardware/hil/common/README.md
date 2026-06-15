# hil.common

Small utilities shared across HIL modules.

| File | Purpose |
| --- | --- |
| `math_utils.py` | Value bounding and angle wrapping helpers. |

This package intentionally stays dependency-light so geometry, configuration,
and node code can all import it without creating circular dependencies.
