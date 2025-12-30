#!/bin/bash
# Attente de la fin du mapping (7 minutes)
echo "⏳ Attente de la fin du mapping (7 minutes)..."
sleep 420

# Sauvegarde de la carte
echo "💾 Sauvegarde de la carte..."
cd /home/ikram/ROS2_Project
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map

# Conversion en PNG
echo "🖼️  Conversion en PNG..."
convert ~/my_robot_map.pgm ~/my_robot_map.png

# Affichage
echo "✅ Carte sauvegardée:"
ls -lh ~/my_robot_map.*
