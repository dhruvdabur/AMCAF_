#!/bin/bash
# Master script to launch Gazebo world, ROS 2 bridge, and the MPC Controller Node together

# Get directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$DIR"

# Launch Gazebo simulation in the background
echo "Starting Gazebo simulation..."
./launch_custom.sh &
GAZEBO_PID=$!

# Wait for Gazebo server to initialize
echo "Waiting for Gazebo to load (5 seconds)..."
sleep 5

# Launch the bridge and the MPC Controller
echo "Starting MPC controller and ROS 2 bridge..."
./run_mpc_control.sh

# Cleanup on exit
cleanup() {
    echo -e "\nShutting down Gazebo and bridge..."
    kill $GAZEBO_PID 2>/dev/null
    pkill -f "ign gazebo"
    pkill -f "ros_gz_bridge"
    exit 0
}
trap cleanup SIGINT SIGTERM
