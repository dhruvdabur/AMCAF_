"""Configuration parsing for hil follower nodes."""

from .ellipse_static import ARMING_SERVICE
from .ellipse_static import CBF_ALPHA_SCALE
from .ellipse_static import CBF_ERROR_SCALE
from .ellipse_static import CBF_GAMMA_SCALE
from .ellipse_static import CBF_R_SAFE_SCALE
from .ellipse_static import COMMAND_TOPIC
from .ellipse_static import HEADING_SCALE
from .ellipse_static import IMAGE_TOPIC
from .ellipse_static import NEUTRAL_VALUE
from .ellipse_static import parse_args
from .ellipse_static import PID_SCALE
from .ellipse_static import PID_WINDOW
from .ellipse_static import print_config
from .ellipse_static import SETTLE_DURATION
from .ellipse_static import UPDATE_RATE_HZ
from .ellipse_static import validate_config
from .ellipse_static import VELOCITY_SCALE

__all__ = [
    'ARMING_SERVICE',
    'CBF_ALPHA_SCALE',
    'CBF_ERROR_SCALE',
    'CBF_GAMMA_SCALE',
    'CBF_R_SAFE_SCALE',
    'COMMAND_TOPIC',
    'HEADING_SCALE',
    'IMAGE_TOPIC',
    'NEUTRAL_VALUE',
    'PID_SCALE',
    'PID_WINDOW',
    'SETTLE_DURATION',
    'UPDATE_RATE_HZ',
    'VELOCITY_SCALE',
    'parse_args',
    'print_config',
    'validate_config',
]
