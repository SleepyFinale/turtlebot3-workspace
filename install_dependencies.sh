#!/bin/bash
# Install all required dependencies for building the workspace

set -e

echo "=========================================="
echo "Installing TurtleBot3 Workspace Dependencies"
echo "=========================================="
echo ""

echo "Updating package list..."
sudo apt update

echo ""
echo "Installing build tools..."
sudo apt install -y git python3-colcon-common-extensions

echo ""
echo "Installing ROS 2 Humble navigation packages..."
# Install essential packages (required for building)
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-backward-ros \
    ros-humble-geographic-msgs \
    ros-humble-test-msgs

echo ""
echo "Installing additional Nav2 development packages..."
# Install additional packages that are commonly needed when building from source
# Note: Some packages may not exist, but we'll try to install what's available
sudo apt install -y \
    ros-humble-nav2-common \
    ros-humble-nav2-msgs \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-core \
    ros-humble-nav2-util \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-velocity-smoother || echo "Some optional packages may not be available (this is OK)"

echo ""
echo "=========================================="
echo "✓ Dependencies installed!"
echo "=========================================="
echo ""
echo "You can now rebuild the workspace:"
echo "  cd ~/turtlebot3_ws"
echo "  ./rebuild_workspace.sh"
