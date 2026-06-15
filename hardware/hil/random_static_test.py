#!/usr/bin/env python3
"""Launch straight_static with randomized surprise obstacles."""

import sys

from .straight_static import main as straight_static_main


DEFAULT_ARGS = [
    '--dry-run',
    '--preview',
    '--no-pid-panel',
    '--controller-mode',
    'pid_velocity_cbf_qp_ellipse',
    '--virtual-vehicle-test',
    '--random-static-obstacles',
    '--debug-visuals',
    '--cbf-a-ell',
    '140',
    '--cbf-b-ell',
    '65',
]


def main(args=None):
    """Run the randomized virtual straight-road CBF-QP test."""
    merged_args = list(DEFAULT_ARGS)
    merged_args.extend(sys.argv[1:] if args is None else args)
    return straight_static_main(merged_args)


if __name__ == '__main__':
    main()
