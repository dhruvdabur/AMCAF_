#!/usr/bin/env python3
"""Benchmark path PID plus velocity control plus a CBF-like safety filter."""

from controller_benchmark_common import PIDVelocityCBFController
from controller_benchmark_common import run_cli


def main():
    """Run the PID plus velocity plus CBF benchmark."""
    run_cli(
        PIDVelocityCBFController,
        "Run the 2D benchmark with PID, velocity control, and CBF filtering.",
    )


if __name__ == "__main__":
    main()
