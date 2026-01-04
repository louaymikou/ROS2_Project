#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/ikram/ROS2_Project/install/setup.bash

# Nettoyer COMPLÈTEMENT l'environnement snap
unset SNAP
unset SNAP_COMMON
unset SNAP_DATA
unset SNAP_INSTANCE_NAME
unset SNAP_LIBRARY_PATH
unset SNAP_NAME
unset SNAP_REAL_HOME
unset SNAP_REVISION
unset SNAP_USER_COMMON
unset SNAP_USER_DATA
unset SNAP_VERSION

# Forcer le path de librairies ROS2 UNIQUEMENT
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/home/ikram/ROS2_Project/install/my_robot_controller/lib

# Lancer RViz2
/opt/ros/humble/bin/rviz2 -d /home/ikram/ROS2_Project/src/my_robot_controller/config/view_bot.rviz
