#!/bin/bash
cd /home/ikram/ROS2_Project
source install/setup.bash

echo "=== Lancement de la navigation complète ==="
echo ""
echo "Ouvrez 3 nouveaux terminaux et exécutez:"
echo ""
echo "Terminal 1 (Gazebo):"
echo "cd /home/ikram/ROS2_Project && source install/setup.bash && LIBGL_ALWAYS_SOFTWARE=1 ros2 launch my_robot_controller launch_sim.launch.py"
echo ""
echo "Terminal 2 (Nav2):"
echo "cd /home/ikram/ROS2_Project && source install/setup.bash && ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/home/ikram/ROS2_Project/my_robot_map.yaml params_file:=/home/ikram/ROS2_Project/src/my_robot_controller/config/nav2_params.yaml"
echo ""
echo "Terminal 3 (RViz):"
echo "cd /home/ikram/ROS2_Project && source install/setup.bash && unset GTK_PATH && unset GTK_MODULES && LIBGL_ALWAYS_SOFTWARE=1 rviz2"
echo ""
