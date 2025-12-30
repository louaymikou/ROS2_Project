#!/bin/bash

cd /home/ikram/ROS2_Project
source install/setup.bash

echo "════════════════════════════════════════════════════"
echo "   LANCEMENT AUTOMATIQUE NAV2"
echo "════════════════════════════════════════════════════"

# Nettoyage
echo "🧹 Nettoyage..."
pkill -9 gzserver gzclient ros2 python3 2>/dev/null
sleep 3

# 1. Gazebo
echo "🚀 Lancement Gazebo..."
ros2 launch my_robot_controller launch_sim.launch.py > /tmp/gazebo_nav.log 2>&1 &
sleep 12
echo "✅ Gazebo lancé"

# 2. Map Server  
echo "🗺️  Lancement Map Server..."
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/my_robot_map2.yaml \
  -p use_sim_time:=true \
  > /tmp/map_server.log 2>&1 &
sleep 3
echo "✅ Map Server lancé"

# 3. AMCL
echo "📍 Lancement AMCL (localisation)..."
ros2 run nav2_amcl amcl --ros-args \
  -p use_sim_time:=true \
  --remap /odom:=/diff_cont/odom \
  > /tmp/amcl.log 2>&1 &
sleep 3
echo "✅ AMCL lancé"

# 4. Lifecycle Manager pour activer map_server et amcl
echo "⚡ Lancement Lifecycle Manager..."
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=true \
  -p autostart:=true \
  -p node_names:='["map_server","amcl"]' \
  > /tmp/lifecycle.log 2>&1 &
sleep 5
echo "✅ Lifecycle Manager lancé"

# 5. Cmd_vel relay (Nav2 publie sur /cmd_vel, robot écoute sur /diff_cont/cmd_vel_unstamped)
echo "🔄 Lancement cmd_vel relay..."
ros2 run topic_tools relay /cmd_vel /diff_cont/cmd_vel_unstamped > /tmp/relay.log 2>&1 &
sleep 2
echo "✅ Relay lancé"

echo ""
echo "════════════════════════════════════════════════════"
echo "   ✅ SYSTÈME PRÊT POUR LA NAVIGATION"
echo "════════════════════════════════════════════════════"
echo ""
echo "Vérifications :"
echo "  ros2 topic list | grep -E '(map|amcl)'"
echo "  ros2 node list"
echo ""
echo "Pour naviguer vers Package 1 :"
echo "  python3 src/my_robot_controller/navigate_to_package.py"
echo ""
echo "Logs disponibles dans /tmp/ :"
echo "  - /tmp/gazebo_nav.log"
echo "  - /tmp/map_server.log"
echo "  - /tmp/amcl.log"
echo "  - /tmp/lifecycle.log"
echo "  - /tmp/relay.log"
