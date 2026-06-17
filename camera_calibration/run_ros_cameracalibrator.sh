#!/usr/bin/env bash
set -euo pipefail

BOARD_SIZE="${BOARD_SIZE:-6x8}"
SQUARE_SIZE="${SQUARE_SIZE:-0.025}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 was not found. Source your ROS 2 Humble environment first:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if ! ros2 pkg executables camera_calibration 2>/dev/null | grep -q 'cameracalibrator'; then
  echo "ROS camera_calibration is not available. Install it with:" >&2
  echo "  sudo apt install ros-humble-camera-calibration" >&2
  exit 1
fi

exec ros2 run camera_calibration cameracalibrator \
  --size "${BOARD_SIZE}" \
  --square "${SQUARE_SIZE}" \
  --ros-args -r "image:=${IMAGE_TOPIC}"
