#!/usr/bin/env python3
"""Compatibility wrapper for the camera calibration test utility."""

import importlib.util
from pathlib import Path


def main():
    """Run the maintained camera test script from camera_calibration."""
    script_path = Path(__file__).resolve().parent / 'camera_calibration' / 'test_camera.py'
    spec = importlib.util.spec_from_file_location('camera_calibration_test_camera', script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.main()


if __name__ == '__main__':
    main()
