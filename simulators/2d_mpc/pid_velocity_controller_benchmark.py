#!/usr/bin/env python3
"""Benchmark path PID plus reference-speed velocity control."""

from controller_benchmark_common import PIDVelocityController
from controller_benchmark_common import run_cli


def main():
    """Run the PID plus velocity-control benchmark."""
    run_cli(
        PIDVelocityController,
        "Run the 2D simulator benchmark with PID plus velocity control.",
    )


if __name__ == "__main__":
    main()
