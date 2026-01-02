#!/bin/bash

# Diagnostic script for SLAM and TF issues
echo "==================================="
echo "ROS2 SLAM Diagnostics"
echo "==================================="
echo ""

echo "1. Checking ROS2 topics..."
echo "-----------------------------------"
ros2 topic list | grep -E "odom|scan|tf"
echo ""

echo "2. Checking /odom topic..."
echo "-----------------------------------"
timeout 2 ros2 topic echo /odom --once 2>/dev/null && echo "✓ /odom is publishing" || echo "✗ /odom is NOT publishing"
echo ""

echo "3. Checking /scan topic..."
echo "-----------------------------------"
timeout 2 ros2 topic echo /scan --once 2>/dev/null && echo "✓ /scan is publishing" || echo "✗ /scan is NOT publishing"
echo ""

echo "4. Checking TF transform (odom → base_link)..."
echo "-----------------------------------"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -5 && echo "✓ TF odom→base_link is available" || echo "✗ TF odom→base_link is NOT available"
echo ""

echo "5. Listing all active nodes..."
echo "-----------------------------------"
ros2 node list
echo ""

echo "==================================="
echo "Diagnostics complete!"
echo "==================================="
