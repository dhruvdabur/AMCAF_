#!/usr/bin/env python3
"""Run all simple-controller benchmarks and print a compact comparison."""

import json

from controller_benchmark_common import BaseTrackController
from controller_benchmark_common import PIDVelocityCBFController
from controller_benchmark_common import PIDVelocityController
from controller_benchmark_common import common_arg_parser
from controller_benchmark_common import simulate
from controller_benchmark_common import write_outputs


CONTROLLERS = (
    BaseTrackController,
    PIDVelocityController,
    PIDVelocityCBFController,
)


def main():
    """Run all benchmark variants with the same arguments."""
    parser = common_arg_parser("Compare PID, PID+velocity, and PID+velocity+CBF.")
    args = parser.parse_args()
    summaries = {}
    for controller_cls in CONTROLLERS:
        summary, rows, history, track, obstacle = simulate(controller_cls, args)
        write_outputs(
            controller_cls.name,
            summary,
            rows,
            history,
            track,
            obstacle,
            args,
        )
        summaries[controller_cls.name] = summary

    print(json.dumps(summaries, indent=2))
    print(f"results directory: {args.output_dir}")


if __name__ == "__main__":
    main()
