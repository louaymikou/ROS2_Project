#!/bin/bash

# ROS2 Nav2 and SLAM Dependencies Installation Script
# Run this script to install all required packages

echo "Installing Nav2 and SLAM dependencies..."
echo "========================================="

# Update package list
echo "Updating package list..."
sudo apt update

# Install Nav2 packages
echo ""
echo "Installing Nav2 Simple Commander..."
sudo apt install -y ros-humble-nav2-simple-commander

echo ""
echo "Installing TF Transformations..."
sudo apt install -y ros-humble-tf-transformations

echo ""
echo "Installing Python Transforms3D..."
sudo apt install -y python3-transforms3d

echo ""
echo "Installing SLAM Toolbox..."
sudo apt install -y ros-humble-slam-toolbox

echo ""
echo "Installing Nav2 Bringup..."
sudo apt install -y ros-humble-nav2-bringup

echo ""
echo "Installing Teleop Twist Keyboard..."
sudo apt install -y ros-humble-teleop-twist-keyboard

echo ""
echo "========================================="
echo "✓ All dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. Build the project: colcon build"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Follow the README.md for SLAM and navigation instructions"
