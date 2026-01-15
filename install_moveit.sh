#!/bin/bash
# MoveIt2 Installation Script for ROS2 Humble

echo "╔══════════════════════════════════════════════════════════╗"
echo "║     MoveIt2 Installation for ROS2 Mobile Manipulator    ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# Check ROS2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not found! Please source ROS2 first:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2 $ROS_DISTRO detected"
echo ""

# Update package list
echo "📦 Updating package list..."
sudo apt update

# Install MoveIt2 core packages
echo ""
echo "🔧 Installing MoveIt2 core packages..."
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-servo

# Install additional dependencies
echo ""
echo "🔧 Installing additional dependencies..."
sudo apt install -y \
    ros-humble-geometric-shapes \
    ros-humble-kdl-parser \
    ros-humble-kdl-kinematics-plugin \
    ros-humble-warehouse-ros-mongo

# Install Python dependencies
echo ""
echo "🐍 Installing Python dependencies..."
pip3 install moveit 2>/dev/null || pip3 install moveit-py

# Build workspace
echo ""
echo "🔨 Building workspace..."
cd ~/ROS2_Project || cd ~/ros2_ws || exit 1

# Clean build
echo "   Cleaning previous build..."
rm -rf build/ install/ log/

# Build
echo "   Building packages..."
colcon build --packages-select my_robot_controller

# Source workspace
source install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║              ✅ INSTALLATION COMPLETE!                   ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "📝 Next steps:"
echo ""
echo "1. Source the workspace:"
echo "   source ~/ROS2_Project/install/setup.bash"
echo ""
echo "2. Test the installation:"
echo "   ros2 launch my_robot_controller moveit.launch.py"
echo ""
echo "3. In another terminal, test arm control:"
echo "   source ~/ROS2_Project/install/setup.bash"
echo "   ros2 run my_robot_controller arm_moveit_control.py home"
echo ""
echo "4. For interactive control:"
echo "   ros2 run my_robot_controller interactive_arm_control.py"
echo ""
echo "📖 Full documentation:"
echo "   docs/MOVEIT_GUIDE.md"
echo "   docs/MOVEIT_QUICK_START.md"
echo ""
