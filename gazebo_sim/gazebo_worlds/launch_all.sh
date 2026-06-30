#!/bin/bash
# Master script to launch Gazebo world, ROS 2 bridge, and the MPC Controller Node together

# Get directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$DIR"

# Launch Gazebo simulation in the background
echo "Starting Gazebo simulation..."
./launch_custom.sh &
GAZEBO_PID=$!

# Wait for Gazebo server to initialize and advertise services
echo "Waiting for Gazebo to initialize..."
sleep 5

# Auto-unpause the simulation
echo "Attempting to unpause Gazebo simulation physics..."
for i in {1..10}; do
    # Call the WorldControl service to unpause physics
    if ign service -s /world/custom_road_world/control \
      --reqtype ignition.msgs.WorldControl \
      --reptype ignition.msgs.Boolean \
      --timeout 1000 \
      --req "pause: false" 2>/dev/null | grep -q "true"; then
        echo "Simulation unpaused successfully."
        break
    fi
    echo "Waiting for Gazebo physics service... (attempt $i/10)"
    sleep 1
done

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
