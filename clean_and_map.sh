#!/bin/bash
# Script pour nettoyer et créer une nouvelle carte propre

echo "╔══════════════════════════════════════════════════════════╗"
echo "║     SCRIPT DE CRÉATION DE CARTE PROPRE                   ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# 1. Arrêter tous les processus ROS
echo "🛑 Arrêt de tous les processus ROS..."
killall -9 gzserver gzclient 2>/dev/null
killall -9 gazebo 2>/dev/null
killall -9 rviz2 2>/dev/null
killall -9 robot_state_publisher 2>/dev/null
killall -9 slam_toolbox 2>/dev/null
sleep 2

# 2. Nettoyer les anciennes cartes
echo "🧹 Nettoyage des anciennes cartes..."
rm -f ~/my_robot_map.yaml ~/my_robot_map.pgm 2>/dev/null
rm -f ~/ROS2_Project/my_robot_map.yaml ~/ROS2_Project/my_robot_map.pgm 2>/dev/null

# 3. Nettoyer les fichiers de pose sauvegardés
echo "🗑️  Suppression des poses sauvegardées..."
rm -f ~/.ros/slam_toolbox/* 2>/dev/null
rm -rf ~/.ros/slam_toolbox 2>/dev/null

echo ""
echo "✅ Nettoyage terminé !"
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║     INSTRUCTIONS POUR CRÉER UNE BONNE CARTE              ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "📋 Étape 1 : Reconstruire le projet"
echo "   cd ~/ROS2_Project"
echo "   colcon build"
echo "   source install/setup.bash"
echo ""
echo "📋 Étape 2 : Lancer la simulation SLAM (Terminal 1)"
echo "   ros2 launch my_robot_controller slam_mapping.launch.py"
echo ""
echo "📋 Étape 3 : Attendre 5 secondes, puis lancer RViz (Terminal 2)"
echo "   cd ~/ROS2_Project"
echo "   source install/setup.bash"
echo "   rviz2"
echo ""
echo "   Configuration RViz :"
echo "   - Fixed Frame: map"
echo "   - Add -> Map -> Topic: /map"
echo "   - Add -> LaserScan -> Topic: /scan"
echo "   - Add -> RobotModel"
echo ""
echo "📋 Étape 4 : Contrôler le robot (Terminal 3)"
echo "   cd ~/ROS2_Project"
echo "   source install/setup.bash"
echo "   python3 src/my_robot_controller/keyboard_controller.py"
echo ""
echo "📋 Étape 5 : Créer la carte LENTEMENT"
echo "   ⚠️  IMPORTANT : Bouger DOUCEMENT !"
echo "   - Vitesse LENTE (appuyer sur 1)"
echo "   - Faire un tour complet de la pièce"
echo "   - S'arrêter tous les 2-3 secondes"
echo "   - Tourner lentement aux coins"
echo "   - Bien couvrir tous les angles"
echo ""
echo "📋 Étape 6 : Sauvegarder la carte (Terminal 4)"
echo "   cd ~/ROS2_Project"
echo "   source install/setup.bash"
echo "   ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map"
echo ""
echo "📋 Étape 7 : Vérifier la carte"
echo "   ls -lh ~/my_robot_map.*"
echo ""
echo "════════════════════════════════════════════════════════════"
echo ""

read -p "Voulez-vous reconstruire maintenant ? (o/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[OoYy]$ ]]
then
    echo "🔨 Reconstruction en cours..."
    cd ~/ROS2_Project
    colcon build --symlink-install
    echo ""
    echo "✅ Reconstruction terminée !"
    echo "💡 Maintenant, suivez les étapes ci-dessus."
fi
