#!/bin/bash
# Script to rebuild the entire workspace

set -e

echo "=========================================="
echo "Rebuilding TurtleBot3 Workspace"
echo "=========================================="
echo ""

cd ~/turtlebot3_ws

# Unset stale environment variables from old install
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS 2 Humble sourced"
else
    echo "ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Check for missing dependencies
echo "Checking for required dependencies..."
MISSING_DEPS=()

# Check for common missing dependencies
if ! dpkg -l | grep -q "ros-humble-backward-ros"; then
    MISSING_DEPS+=("ros-humble-backward-ros")
fi

if ! dpkg -l | grep -q "ros-humble-geographic-msgs"; then
    MISSING_DEPS+=("ros-humble-geographic-msgs")
fi

if ! dpkg -l | grep -q "ros-humble-test-msgs"; then
    MISSING_DEPS+=("ros-humble-test-msgs")
fi

if ! dpkg -l | grep -q "ros-humble-navigation2"; then
    MISSING_DEPS+=("ros-humble-navigation2")
fi

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo ""
    echo "⚠ Missing dependencies detected:"
    for dep in "${MISSING_DEPS[@]}"; do
        echo "  - $dep"
    done
    echo ""
    read -p "Install missing dependencies now? [y/N]: " install_deps
    if [[ "$install_deps" =~ ^[Yy]$ ]]; then
        echo "Installing dependencies..."
        sudo apt update
        sudo apt install -y "${MISSING_DEPS[@]}"
        echo "✓ Dependencies installed"
    else
        echo "⚠ Continuing without installing dependencies. Build may fail."
    fi
    echo ""
fi

# Ask user if they want a clean rebuild
echo "Choose rebuild option:"
echo "  1) Clean rebuild (removes build/install/log, then rebuilds) - Recommended for first time or major changes"
echo "  2) Incremental rebuild (faster, keeps existing build artifacts)"
echo ""
read -p "Enter choice [1 or 2] (default: 2): " choice
choice=${choice:-2}

if [ "$choice" = "1" ]; then
    echo ""
    echo "Cleaning workspace (removing build, install, log directories)..."
    rm -rf build install log
    echo "✓ Cleaned workspace"
    echo ""
    echo "Building workspace from scratch..."
    colcon build --symlink-install
else
    echo ""
    echo "Building workspace (incremental)..."
    colcon build --symlink-install
fi

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build successful!"
    echo "=========================================="
    echo ""
    echo "Source the workspace:"
    echo "  source ~/turtlebot3_ws/install/setup.bash"
    echo ""
    echo "Or add to your ~/.bashrc:"
    echo "  echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "✗ Build failed!"
    echo "=========================================="
    echo ""
    echo "Check the error messages above for details."
    echo "Common issues:"
    echo "  - Missing dependencies: sudo apt install ros-humble-<package>"
    echo "  - Build errors in specific packages: check package-specific errors"
    exit 1
fi
