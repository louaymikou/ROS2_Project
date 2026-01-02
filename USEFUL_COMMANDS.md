# 📋 Commandes Utiles - Intégration IMU

## 🔧 Installation et Build

```bash
# Installer robot_localization
sudo apt install ros-humble-robot-localization

# Build le workspace
cd ~/ROS_PROJECT
colcon build --packages-select my_robot_controller

# Source
source install/setup.bash
```

## 🚀 Lancement

```bash
# Simulation complète avec IMU
ros2 launch my_robot_controller launch_sim.launch.py

# SLAM avec fusion IMU
ros2 launch my_robot_controller slam_mapping.launch.py

# EKF seulement (si besoin)
ros2 launch my_robot_controller robot_localization.launch.py
```

## 🔍 Vérification et Monitoring

### Vérifier que tout fonctionne
```bash
# Test automatique complet
./test_imu_integration.sh

# Vérifier les nodes actifs
ros2 node list | grep ekf
# Devrait afficher: /ekf_filter_node

# Vérifier les topics
ros2 topic list | grep -E "imu|odom"
# Devrait afficher:
# /imu/data
# /diff_cont/odom  
# /odometry/local
```

### Monitorer les fréquences
```bash
# Fréquence IMU (devrait être ~100 Hz)
ros2 topic hz /imu/data

# Fréquence odométrie roues (devrait être ~50 Hz)
ros2 topic hz /diff_cont/odom

# Fréquence odométrie fusionnée (devrait être ~50 Hz)
ros2 topic hz /odometry/local
```

### Voir les données en temps réel
```bash
# Données IMU brutes
ros2 topic echo /imu/data

# Odométrie des roues
ros2 topic echo /diff_cont/odom

# Odométrie fusionnée (EKF)
ros2 topic echo /odometry/local

# Comparer les deux odométries (script personnalisé)
ros2 run my_robot_controller compare_odometry.py
```

## 📊 Analyse et Debug

### Transformations TF
```bash
# Générer le graphe TF
ros2 run tf2_tools view_frames

# Ouvrir le PDF généré
evince frames.pdf

# Voir une transformation en temps réel
ros2 run tf2_ros tf2_echo odom base_link

# Voir la transformation vers l'IMU
ros2 run tf2_ros tf2_echo base_link imu_link
```

### Paramètres de l'EKF
```bash
# Lister tous les paramètres
ros2 param list /ekf_filter_node

# Voir la fréquence
ros2 param get /ekf_filter_node frequency

# Voir la config odom0
ros2 param get /ekf_filter_node odom0

# Voir la config imu0
ros2 param get /ekf_filter_node imu0

# Changer la fréquence (temporaire)
ros2 param set /ekf_filter_node frequency 60.0
```

### Informations sur les nodes
```bash
# Info complète sur l'EKF
ros2 node info /ekf_filter_node

# Info sur robot_state_publisher
ros2 node info /robot_state_publisher

# Graphe des nodes
rqt_graph
```

## 📈 Analyse de Performance

### Latence
```bash
# Mesurer la latence (si disponible)
ros2 topic delay /odometry/local
```

### Covariance
```bash
# Voir la covariance de position (incertitude)
ros2 topic echo /odometry/local --field pose.covariance

# Voir uniquement la position
ros2 topic echo /odometry/local --field pose.pose.position

# Voir uniquement l'orientation
ros2 topic echo /odometry/local --field pose.pose.orientation
```

### Enregistrement de données
```bash
# Enregistrer 60 secondes de données
ros2 bag record -d 60 /imu/data /diff_cont/odom /odometry/local /cmd_vel

# Enregistrer tout (attention à la taille)
ros2 bag record -a -d 30

# Rejouer un bag
ros2 bag play <nom_du_bag>

# Info sur un bag
ros2 bag info <nom_du_bag>
```

## 🎮 Tests de Mouvement

### Commandes de test basiques
```bash
# Avancer en ligne droite
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

# Rotation sur place
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Trajectoire courbe
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"

# Arrêt
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Tests avec téléopération
```bash
# Dans un terminal: lancer la simulation
ros2 launch my_robot_controller launch_sim.launch.py

# Dans un autre terminal: téléopération clavier
ros2 run my_robot_controller keyboard_controller.py

# Ou avec PS4 (si disponible)
ros2 run my_robot_controller ps4_controller.py
```

## 🛠️ Diagnostic et Dépannage

### L'EKF ne démarre pas
```bash
# Vérifier que robot_localization est installé
ros2 pkg list | grep robot_localization

# Si absent, installer:
sudo apt install ros-humble-robot-localization

# Vérifier les logs
ros2 run robot_localization ekf_node --ros-args --log-level debug
```

### Pas de données IMU
```bash
# Vérifier que l'IMU est dans le URDF
ros2 param get /robot_state_publisher robot_description | grep imu_link

# Vérifier dans Gazebo
gz topic -l | grep imu

# Tester la publication
ros2 topic echo /imu/data --once
```

### L'odométrie fusionnée n'est pas publiée
```bash
# Vérifier les logs de l'EKF
ros2 topic echo /rosout | grep ekf

# Vérifier que les sources sont actives
ros2 topic hz /imu/data
ros2 topic hz /diff_cont/odom

# Redémarrer l'EKF
ros2 lifecycle set /ekf_filter_node configure
ros2 lifecycle set /ekf_filter_node activate
```

## 📝 Édition et Ajustement

### Éditer la configuration EKF
```bash
# Ouvrir avec VS Code
code ~/ROS_PROJECT/src/my_robot_controller/config/ekf_params.yaml

# Ou avec nano
nano ~/ROS_PROJECT/src/my_robot_controller/config/ekf_params.yaml

# Après modification, rebuild
cd ~/ROS_PROJECT
colcon build --packages-select my_robot_controller
source install/setup.bash
```

### Éditer la configuration Nav2
```bash
code ~/ROS_PROJECT/src/my_robot_controller/config/nav2_params.yaml
```

### Éditer l'IMU dans le URDF
```bash
code ~/ROS_PROJECT/src/my_robot_controller/description/imu.xacro
```

## 📚 Visualisation

### RViz2
```bash
# Lancer RViz2
rviz2

# Dans RViz2, ajouter:
# - TF (voir les frames)
# - Imu (/imu/data)
# - Odometry (/odometry/local)
# - Odometry (/diff_cont/odom) pour comparer
# - LaserScan (/scan)
```

### Plotjuggler (graphiques en temps réel)
```bash
# Installer si nécessaire
sudo apt install ros-humble-plotjuggler-ros

# Lancer
ros2 run plotjuggler plotjuggler

# Dans l'interface:
# 1. Start streaming
# 2. Sélectionner ROS2 Topic Subscriber
# 3. Ajouter /odometry/local et /diff_cont/odom
# 4. Glisser-déposer les données dans les graphiques
```

## 🧹 Nettoyage

```bash
# Nettoyer les builds
cd ~/ROS_PROJECT
rm -rf build/ install/ log/

# Rebuild propre
colcon build --packages-select my_robot_controller
```

## 💾 Sauvegarde de la Configuration

```bash
# Exporter la configuration actuelle
ros2 param dump /ekf_filter_node > ekf_params_backup.yaml

# Comparer avec la version originale
diff ekf_params_backup.yaml src/my_robot_controller/config/ekf_params.yaml
```

## 📖 Aide et Documentation

```bash
# Voir l'aide de robot_localization
ros2 run robot_localization ekf_node --help

# Documentation en ligne
firefox http://docs.ros.org/en/humble/p/robot_localization/

# Documentation locale
cat ~/ROS_PROJECT/docs/IMU_INTEGRATION_GUIDE.md
cat ~/ROS_PROJECT/docs/EKF_TUNING_GUIDE.md
cat ~/ROS_PROJECT/docs/IMU_ARCHITECTURE.md
```

---

**💡 Astuce:** Ajoutez ces commandes en alias dans votre `~/.bashrc` pour un accès rapide !

```bash
# Ajouter à ~/.bashrc
alias ros2_ws='cd ~/ROS_PROJECT && source install/setup.bash'
alias ros2_sim='ros2 launch my_robot_controller launch_sim.launch.py'
alias ros2_test_imu='~/ROS_PROJECT/test_imu_integration.sh'
alias ros2_compare_odom='ros2 run my_robot_controller compare_odometry.py'
```
