#!/bin/bash
# Install all required system dependencies for turtlebot3_ws

echo "=========================================="
echo "Installing System Dependencies"
echo "=========================================="
echo ""

# Update package list
echo "Updating package list..."
sudo apt update

# Install required dependencies
echo ""
echo "Installing build dependencies..."
sudo apt install -y \
    libyaml-cpp-dev \
    libeigen3-dev \
    libgraphicsmagick++1-dev \
    uuid-dev \
    python3-colcon-common-extensions \
    ros-humble-tf2-sensor-msgs

echo ""
echo "=========================================="
echo "✓ Dependencies installed successfully!"
echo "=========================================="
echo ""
echo "You can now run: ./clean_rebuild.sh"
