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
pkill -f "mpc_controller_node.py"

cleanup() {
    echo -e "\nShutting down..."
    if [ -n "${BRIDGE_PID:-}" ]; then
        kill "$BRIDGE_PID" 2>/dev/null
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
  > /tmp/ros_gz_bridge.log 2>&1 &

BRIDGE_PID=$!
sleep 2

# Check if bridge started successfully
if ps -p $BRIDGE_PID > /dev/null; then
    echo "Bridge started successfully (PID: $BRIDGE_PID)."
else
    echo "Error: Failed to start ros_gz_bridge. Check /tmp/ros_gz_bridge.log for details."
    exit 1
fi

echo "Launching MPC Controller Node..."
# Export PYTHONPATH so the node can find the hardware.hil.controllers packages
export PYTHONPATH="$REPO_ROOT:$PYTHONPATH"

# Launch the node
python3 "$SCRIPT_DIR/mpc_controller_node.py"

# Cleanup on exit
cleanup
