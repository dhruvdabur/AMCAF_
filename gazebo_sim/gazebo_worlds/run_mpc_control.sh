#!/bin/bash
# Helper script to run the ROS 2 - Gazebo bridge and the MPC Controller Node

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Ensure ROS 2 Humble environment is sourced (usually in ~/.bashrc, but safety first)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash"
fi
if [ -f "$REPO_ROOT/hardware/install/setup.bash" ]; then
    source "$REPO_ROOT/hardware/install/setup.bash"
fi

# Kill any existing bridge or node to prevent port/name conflicts
pkill -f "ros_gz_bridge"
pkill -f "ros_ign_bridge"
pkill -f "aruco_merge.py"
pkill -f "mpc_controller_node.py"

# Clean up stale JSON files
rm -f /tmp/aruco_pose.json

cleanup() {
    echo -e "\nShutting down..."
    if [ -n "${BRIDGE_PID:-}" ]; then
        kill "$BRIDGE_PID" 2>/dev/null
    fi
    if [ -n "${LIDAR_BRIDGE_PID:-}" ]; then
        kill "$LIDAR_BRIDGE_PID" 2>/dev/null
    fi
    if [ -n "${ARUCO_MERGE_PID:-}" ]; then
        kill "$ARUCO_MERGE_PID" 2>/dev/null
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Starting ROS 2 - Gazebo parameter bridge..."
# Bridge /clock and /model/prius/odometry from Gazebo -> ROS, and cmd_vel from ROS -> Gazebo.
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
  /model/prius/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /model/prius/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
  /world/custom_road_world/set_pose@ros_gz_interfaces/srv/SetEntityPose \
  > /tmp/ros_gz_bridge.log 2>&1 &

BRIDGE_PID=$!
sleep 1

# Launch additional Ignition bridge for 2D LiDAR as requested
ros2 run ros_ign_bridge parameter_bridge \
  "/lidar2D/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan" \
  > /tmp/ros_ign_bridge.log 2>&1 &

LIDAR_BRIDGE_PID=$!
sleep 1

# Check if bridges started successfully
if ps -p $BRIDGE_PID > /dev/null && ps -p $LIDAR_BRIDGE_PID > /dev/null; then
    echo "Bridges started successfully (GZ PID: $BRIDGE_PID, IGN PID: $LIDAR_BRIDGE_PID)."
else
    echo "Error: Failed to start parameter bridges. Check /tmp/ros_gz_bridge.log and /tmp/ros_ign_bridge.log for details."
    exit 1
fi

echo "Launching MPC Controller Node..."
# Export PYTHONPATH so the node can find the hardware.hil.controllers packages
export PYTHONPATH="$REPO_ROOT:$PYTHONPATH"

# Conditionally launch aruco_merge.py if requested
if [ "${USE_ARUCO}" = "true" ] || [ "${USE_ARUCO}" = "1" ]; then
    echo "ArUco pose feedback enabled. Launching aruco_merge.py..."
    export USE_ARUCO="true"
    
    # If no custom ARUCO_ARGS provided, fallback to defaults
    if [ -z "${ARUCO_ARGS:-}" ]; then
        ARUCO_ARGS="--cam1 /dev/video0 --cam2 /dev/video4 --intrinsics1 /home/monukoru/aruco_merge/camera_calibration/cam1.yaml --intrinsics2 /home/monukoru/aruco_merge/camera_calibration/cam2.yaml"
    fi
    
    echo "Running: python3 /home/monukoru/aruco_merge/monocular_merge/aruco_merge.py --json --output-json /tmp/aruco_pose.json $ARUCO_ARGS"
    python3 /home/monukoru/aruco_merge/monocular_merge/aruco_merge.py --json --output-json /tmp/aruco_pose.json $ARUCO_ARGS > /tmp/aruco_merge.log 2>&1 &
    ARUCO_MERGE_PID=$!
    sleep 1
fi

# Launch the node
python3 "$SCRIPT_DIR/mpc_controller_node.py"

# Cleanup on exit
cleanup
