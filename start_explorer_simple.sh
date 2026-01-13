#!/bin/bash
# Simple explorer launcher - starts with Nav2 costmap config
# The explorer will wait for the costmap automatically

# Set ROS_DOMAIN_ID if not already set (defaults to 0 if not set)
if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=30
    echo "Note: ROS_DOMAIN_ID not set, using default: 30"
    echo "  To make permanent, add to ~/.bashrc: export ROS_DOMAIN_ID=30"
fi

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting explorer with Nav2 costmap configuration..."
echo "Explorer will wait for /global_costmap/costmap to become available."
echo ""

# Use Nav2 costmap (recommended when Nav2 is running)
ros2 run explore_lite explore --ros-args \
    --params-file src/m-explore-ros2/explore/config/params_costmap.yaml
