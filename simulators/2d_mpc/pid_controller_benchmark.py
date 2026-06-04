#!/usr/bin/env python3
"""Benchmark path tracking with steering PID and fixed speed control."""

from controller_benchmark_common import BaseTrackController
from controller_benchmark_common import run_cli


def main():
    """Run the PID-only benchmark."""
    run_cli(
        BaseTrackController,
        "Run the 2D simulator benchmark with PID-only path tracking.",
    )


if __name__ == "__main__":
    main()
