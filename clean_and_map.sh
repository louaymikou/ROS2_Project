#!/bin/bash

# Script de mapping complet automatique
cd /home/ikram/ROS2_Project
source install/setup.bash

echo "════════════════════════════════════════════════════"
echo "   NETTOYAGE ET MAPPING AUTOMATIQUE"
echo "════════════════════════════════════════════════════"

# 1. Nettoyage complet
echo "🧹 Nettoyage des anciens processus..."
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 ros2 2>/dev/null
pkill -9 python3 2>/dev/null
sleep 3

# Suppression anciennes cartes
rm -f ~/my_robot_map.* /home/ikram/ROS2_Project/my_robot_map.png 2>/dev/null
echo "✅ Nettoyage terminé"
echo ""

# 2. Lancement Gazebo
echo "🚀 Lancement de Gazebo..."
ros2 launch my_robot_controller launch_sim.launch.py > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
sleep 12
echo "✅ Gazebo lancé (PID: $GAZEBO_PID)"
echo ""

# 3. Lancement SLAM avec remapping
echo "🗺️  Lancement SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file src/my_robot_controller/config/slam_params.yaml \
  -p use_sim_time:=true \
  --remap /odom:=/diff_cont/odom \
  > /tmp/slam.log 2>&1 &
SLAM_PID=$!
sleep 8
echo "✅ SLAM lancé (PID: $SLAM_PID)"
echo ""

# Vérification du topic /map
echo "🔍 Vérification du topic /map..."
if ros2 topic list | grep -q "^/map$"; then
    echo "✅ Topic /map détecté !"
else
    echo "❌ ERREUR : Topic /map non trouvé !"
    echo "Logs SLAM :"
    tail -20 /tmp/slam.log
    exit 1
fi
echo ""

# 4. Lancement du mapper
echo "🤖 Lancement du Wall Follower Mapper..."
python3 src/my_robot_controller/wall_follower_mapper.py > /tmp/mapper.log 2>&1 &
MAPPER_PID=$!
echo "✅ Mapper lancé (PID: $MAPPER_PID)"
echo ""

# 5. Surveillance du mapper
echo "⏳ Mapping en cours (environ 7 minutes)..."
echo "   - Scan 360° initial : 30s"
echo "   - Premier tour : 2.5 min"
echo "   - Scan 360° intermédiaire : 30s"
echo "   - Second tour : 2.5 min"
echo "   - Scan 360° final : 30s"
echo ""

# Attendre la fin du mapper
sleep 420  # 7 minutes

# Vérifier si le mapper tourne encore
if ps -p $MAPPER_PID > /dev/null 2>&1; then
    echo "⚠️  Mapper encore actif, attente de 1 minute supplémentaire..."
    sleep 60
fi

echo ""
echo "════════════════════════════════════════════════════"
echo "   SAUVEGARDE DE LA CARTE"
echo "════════════════════════════════════════════════════"

# 6. Sauvegarde de la carte
echo "💾 Sauvegarde de la carte..."
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map
SAVE_RESULT=$?

if [ $SAVE_RESULT -eq 0 ]; then
    echo "✅ Carte sauvegardée avec succès !"
    
    # Conversion en PNG
    if command -v convert &> /dev/null; then
        echo "🖼️  Conversion en PNG..."
        convert ~/my_robot_map.pgm ~/my_robot_map.png
        cp ~/my_robot_map.png /home/ikram/ROS2_Project/
        echo "✅ Image PNG créée !"
    fi
    
    echo ""
    echo "📂 Fichiers créés :"
    ls -lh ~/my_robot_map.* 2>/dev/null
    ls -lh /home/ikram/ROS2_Project/my_robot_map.png 2>/dev/null
    
else
    echo "❌ ERREUR lors de la sauvegarde !"
    echo "Logs SLAM :"
    tail -30 /tmp/slam.log
fi

echo ""
echo "════════════════════════════════════════════════════"
echo "   MAPPING TERMINÉ"
echo "════════════════════════════════════════════════════"
echo ""
echo "Pour voir la carte : ouvrir /home/ikram/ROS2_Project/my_robot_map.png"
echo "Pour arrêter tout : pkill -9 gzserver; pkill -9 gzclient; pkill -9 ros2; pkill -9 python3"
