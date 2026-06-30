#!/bin/bash
# Helper script to run the ROS 2 - Gazebo bridge and the MPC Controller Node

# Ensure ROS 2 Humble environment is sourced (usually in ~/.bashrc, but safety first)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Kill any existing bridge or node to prevent port/name conflicts
pkill -f "ros_gz_bridge"
pkill -f "mpc_controller_node.py"

echo "Starting ROS 2 - Gazebo parameter bridge..."
# Bridge /model/prius/cmd_vel (ROS -> Gazebo) and /model/prius/odometry (Gazebo -> ROS)
ros2 run ros_gz_bridge parameter_bridge \
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
export PYTHONPATH=/home/dhruv/.local/lib/python3.10/site-packages:/home/dhruv/amcaf:$PYTHONPATH

# Launch the node
python3 /home/dhruv/amcaf/gazebo_sim/gazebo_worlds/mpc_controller_node.py

# Cleanup on exit
cleanup() {
    echo -e "\nShutting down..."
    kill $BRIDGE_PID 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM
