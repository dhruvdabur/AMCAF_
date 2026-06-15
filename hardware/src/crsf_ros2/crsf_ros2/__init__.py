"""crsf_ros2 package root.

The HIL package is kept at ``hardware/hil`` so simulation code can live at the
hardware workspace level while preserving imports such as ``crsf_ros2.hil``.
"""

from pathlib import Path


_hardware_dir = Path(__file__).resolve().parents[3]
_hardware_dir_text = str(_hardware_dir)
if (_hardware_dir / 'hil').is_dir() and _hardware_dir_text not in __path__:
    __path__.append(_hardware_dir_text)
