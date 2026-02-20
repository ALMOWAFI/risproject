#!/bin/bash
# Setup script for Memory Game ROS1 workspace
# Run this script to set up your workspace from scratch

set -e  # Exit on error

echo "=========================================="
echo "Memory Game ROS1 Workspace Setup"
echo "=========================================="

# Check ROS installation
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS is not sourced!"
    echo "Please run: source /opt/ros/noetic/setup.bash (or melodic)"
    exit 1
fi

echo "ROS Distribution: $ROS_DISTRO"

# Create workspace if it doesn't exist
WORKSPACE_DIR="$HOME/catkin_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Creating workspace at $WORKSPACE_DIR..."
    mkdir -p "$WORKSPACE_DIR/src"
    cd "$WORKSPACE_DIR/src"
    catkin_init_workspace
else
    echo "Workspace already exists at $WORKSPACE_DIR"
fi

# Navigate to workspace
cd "$WORKSPACE_DIR/src"

# Check if package already exists
if [ -d "memory_game" ]; then
    echo "Package 'memory_game' already exists."
    read -p "Do you want to remove it and re-clone? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf memory_game
        echo "Removed existing package"
    else
        echo "Keeping existing package. Exiting."
        exit 0
    fi
fi

# Clone repository (if using git)
echo "Please clone the repository manually:"
echo "  cd $WORKSPACE_DIR/src"
echo "  git clone <repository-url> memory_game"
echo ""
read -p "Press Enter after cloning the repository..."

# Check if package exists now
if [ ! -d "memory_game" ]; then
    echo "ERROR: memory_game package not found!"
    echo "Please clone the repository first."
    exit 1
fi

# Install dependencies
echo "Installing dependencies..."
cd "$WORKSPACE_DIR"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
echo "Building workspace..."
cd "$WORKSPACE_DIR"
catkin_make

# Source workspace
echo "Sourcing workspace..."
source devel/setup.bash

# Verify installation
echo ""
echo "=========================================="
echo "Verification"
echo "=========================================="
if rospack find memory_game > /dev/null 2>&1; then
    echo "✓ Package found: $(rospack find memory_game)"
else
    echo "✗ Package not found!"
    exit 1
fi

if rostopic list > /dev/null 2>&1; then
    echo "✓ ROS master connection OK"
else
    echo "⚠ ROS master not running (this is OK if you haven't started roscore)"
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Source the workspace: source ~/catkin_ws/devel/setup.bash"
echo "2. Add to ~/.bashrc for permanent sourcing"
echo "3. Start roscore: roscore"
echo "4. Launch the system: roslaunch memory_game full_system.launch"
echo ""
echo "For more information, see:"
echo "  - README.md"
echo "  - ROS1_GUIDE.md"
echo "  - COLLABORATION.md"
echo ""
