#!/bin/bash

cd /home/ikram/ROS2_Project
source install/setup.bash

echo "════════════════════════════════════════════════════"
echo "   LANCEMENT NAV2 COMPLET POUR NAVIGATION"
echo "════════════════════════════════════════════════════"

# Nettoyage
echo "🧹 Nettoyage..."
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 ros2 2>/dev/null
pkill -9 python3 2>/dev/null
sleep 3

# 1. Gazebo
echo ""
echo "🚀 1/6 - Lancement Gazebo..."
ros2 launch my_robot_controller launch_sim.launch.py > /tmp/nav_gazebo.log 2>&1 &
sleep 12
echo "   ✅ Gazebo lancé"

# 2. Map Server + Lifecycle
echo ""
echo "🗺️  2/6 - Lancement Map Server..."
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/my_robot_map2.yaml \
  -p use_sim_time:=true \
  > /tmp/nav_map_server.log 2>&1 &
sleep 2

ros2 lifecycle set /map_server configure > /dev/null 2>&1
ros2 lifecycle set /map_server activate > /dev/null 2>&1
echo "   ✅ Map Server activé"

# 3. AMCL + Lifecycle
echo ""
echo "📍 3/6 - Lancement AMCL (localisation)..."
ros2 run nav2_amcl amcl --ros-args \
  -p use_sim_time:=true \
  --remap /odom:=/diff_cont/odom \
  > /tmp/nav_amcl.log 2>&1 &
sleep 3

ros2 lifecycle set /amcl configure > /dev/null 2>&1
ros2 lifecycle set /amcl activate > /dev/null 2>&1
echo "   ✅ AMCL activé"

# 4. Nav2 Bringup (Controller, Planner, Behavior, etc.)
echo ""
echo "🎯 4/6 - Lancement Nav2 Stack (Controller + Planner)..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=src/my_robot_controller/config/nav2_params.yaml \
  > /tmp/nav_stack.log 2>&1 &
sleep 10
echo "   ✅ Nav2 Stack lancé"

# 5. Cmd_vel relay
echo ""
echo "🔄 5/6 - Lancement cmd_vel relay..."
ros2 run topic_tools relay /cmd_vel /diff_cont/cmd_vel_unstamped > /tmp/nav_relay.log 2>&1 &
sleep 2
echo "   ✅ Relay cmd_vel lancé"

# 6. Pose initiale
echo ""
echo "📍 6/6 - Définition de la pose initiale..."
sleep 2
python3 src/my_robot_controller/set_initial_pose.py
echo "   ✅ Pose initiale définie"

echo ""
echo "════════════════════════════════════════════════════"
echo "   ✅ NAV2 COMPLÈTEMENT PRÊT !"
echo "════════════════════════════════════════════════════"
echo ""
echo "📊 Vérifications disponibles :"
echo "  ros2 topic list | grep -E '(map|cmd_vel|goal)'"
echo "  ros2 node list"
echo ""
echo "🎯 Pour naviguer vers Package 1 :"
echo "  cd ~/ROS2_Project && source install/setup.bash"
echo "  python3 src/my_robot_controller/navigate_to_package.py"
echo ""
echo "📝 Logs disponibles :"
echo "  /tmp/nav_gazebo.log"
echo "  /tmp/nav_map_server.log"
echo "  /tmp/nav_amcl.log"
echo "  /tmp/nav_stack.log"
echo "  /tmp/nav_relay.log"
echo ""
