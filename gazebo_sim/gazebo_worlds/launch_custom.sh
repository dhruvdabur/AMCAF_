#!/usr/bin/env bash
export IGN_GAZEBO_RESOURCE_PATH=/home/dhruv/amcaf/gazebo_sim/custom_worlds/models:$IGN_GAZEBO_RESOURCE_PATH
pkill -f "ign gazebo"
ign gazebo -v 4 /home/dhruv/amcaf/gazebo_sim/gazebo_worlds/custom_road.sdf
