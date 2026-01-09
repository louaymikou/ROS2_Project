#!/bin/bash

echo "======================================"
echo "🚀 LANCEMENT SYSTÈME COMPLET"
echo "======================================"

# Charger l'environnement
source /home/ikram/ROS2_Project/install/setup.bash

# Nettoyer
killall -9 gzserver gzclient ruby ros2 rviz2 2>/dev/null
sleep 2

echo ""
echo "▶ Lancement de Gazebo + Robot + Controllers..."
ros2 launch my_robot_controller launch_complete_system.launch.py &
LAUNCH_PID=$!
sleep 15

echo ""
echo "▶ Lancement Nav2 + AMCL + Map Server..."
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/ikram/ROS2_Project/my_robot_map.yaml \
  use_sim_time:=true \
  params_file:=/home/ikram/ROS2_Project/src/my_robot_controller/config/nav2_params.yaml &
NAV2_PID=$!
sleep 10

echo ""
echo "▶ Lancement du relay cmd_vel → diff_cont..."
ros2 run topic_tools relay /cmd_vel /diff_cont/cmd_vel_unstamped &
RELAY_PID=$!
sleep 3

echo ""
echo "▶ Publication de la pose initiale..."
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, 
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, 
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}" \
  --once
sleep 3

echo ""
echo "======================================"
echo "✅ SYSTÈME COMPLÈTEMENT LANCÉ!"
echo "======================================"
echo ""
echo "📊 Pour vérifier:"
echo "  ros2 topic list | grep -E 'map|odom'"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo ""
echo "🎯 Pour naviguer:"
echo "  ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: \"map\"}, pose: {position: {x: 2.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}' --once"
echo ""
echo "⏹  Pour arrêter:"
echo "  killall -9 gzserver gzclient ruby ros2"
echo ""

# Garder le script actif
wait
