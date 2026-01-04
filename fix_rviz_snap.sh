#!/bin/bash

echo "🔧 Solution 1: Lancer RViz2 avec LD_PRELOAD forcé"
echo "=============================================="

# Forcer l'utilisation de la librairie système au lieu de snap
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/ikram/ROS2_Project/install/setup.bash

# Lancer RViz2
echo "Lancement de RViz2..."
/opt/ros/humble/bin/rviz2 -d /home/ikram/ROS2_Project/src/my_robot_controller/config/view_bot.rviz
