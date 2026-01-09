# 🤖 ROS2 Mobile Manipulator Project

Mobile robot with manipulator arm, SLAM mapping, autonomous navigation, and **IMU sensor fusion** for improved odometry.

## ✨ Fonctionnalités Principales

- 🗺️ **Mapping SLAM** - Création de cartes avec SLAM Toolbox
- 🧭 **Navigation Autonome** - Navigation avec Nav2
- 🎮 **Contrôle Téléopéré** - Clavier et manette PS4
- 🦾 **Bras Manipulateur** - Bras 3-DOF avec pince
- 📡 **Fusion de Capteurs** - IMU + Wheel Encoders avec EKF
- 🎯 **Simulation Gazebo** - Environnement réaliste

## 🚀 Démarrage Rapide

### 📖 Documentation Disponible

| Guide | Description |
|-------|-------------|
| **[QUICK_START.md](QUICK_START.md)** | ⚡ Commandes rapides pour démarrer |
| **[PROJECT_GUIDE.md](PROJECT_GUIDE.md)** | 📚 Guide complet et détaillé |
| **[check_system.sh](check_system.sh)** | 🔍 Script de vérification du système |

### ⚙️ Installation Rapide

```bash
# Installer les dépendances ROS2
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-robot-localization \
                 ros-humble-gazebo-ros-pkgs

# Builder le projet
cd ~/ROS2_Project
colcon build
source install/setup.bash

# Vérifier le système
./check_system.sh
```

## 🗺️ Mapping (Créer une Carte)

```bash
# Terminal 1: Lancer Gazebo + SLAM
ros2 launch my_robot_controller slam_mapping.launch.py

# Terminal 2: Visualisation
rviz2

# Terminal 3: Contrôle clavier
python3 src/my_robot_controller/nodes/controllers/keyboard_controller.py

# Terminal 4: Sauvegarder la carte
ros2 run nav2_map_server map_saver_cli -f src/my_robot_controller/maps/my_robot_map
```

## 🧭 Navigation Autonome

```bash
# Terminal 1: Lancer Gazebo + Nav2
ros2 launch my_robot_controller navigation.launch.py

# Terminal 2: Navigation autonome
ros2 run my_robot_controller auto_navigator.py demo

# Terminal 3: RViz Nav2
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## 🔧 Intégration IMU

✨ **Fusion de capteurs avec robot_localization**
- EKF fusion: Wheel Encoders + IMU
- Amélioration de la précision d'orientation
- Détection de glissement
- Topic fusionné: `/odometry/local`

📖 Voir [docs/IMU_INTEGRATION_GUIDE.md](docs/IMU_INTEGRATION_GUIDE.md) pour les détails

## 📁 Structure du Projet

```
ROS2_Project/
├── 📄 PROJECT_GUIDE.md          # Guide complet du projet
├── 📄 QUICK_START.md            # Guide de démarrage rapide
├── 🔍 check_system.sh           # Script de vérification
├── docs/                        # Documentation détaillée
│   ├── IMU_INTEGRATION_GUIDE.md
│   ├── EKF_TUNING_GUIDE.md
│   └── INSTALLATION_GUIDE.md
├── src/my_robot_controller/
│   ├── config/                  # Configuration
│   │   ├── slam_params.yaml
│   │   ├── nav2_params.yaml
│   │   └── ekf_params.yaml
│   ├── description/             # URDF/Xacro
│   │   ├── robot.urdf.xacro
│   │   └── imu.xacro
│   ├── launch/                  # Fichiers de lancement
│   │   ├── slam_mapping.launch.py
│   │   ├── navigation.launch.py
│   │   └── robot_localization.launch.py
│   ├── nodes/                   # Scripts Python
│   │   ├── controllers/
│   │   │   └── keyboard_controller.py
│   │   └── navigation/
│   │       └── auto_navigator.py
│   ├── maps/                    # Cartes sauvegardées
│   │   ├── my_robot_map.yaml
│   │   └── my_robot_map.pgm
│   ├── models/                  # Modèles 3D
│   └── worlds/                  # Mondes Gazebo
├── build/                       # Fichiers de build
├── install/                     # Installation
└── log/                         # Logs
```

## 🎯 Workflows Principaux

### 1️⃣ Première utilisation: Mapping
```bash
./check_system.sh               # Vérifier le système
# Puis suivre QUICK_START.md section "MAPPING"
```

### 2️⃣ Utilisations suivantes: Navigation
```bash
./check_system.sh               # Vérifier (carte doit exister)
# Puis suivre QUICK_START.md section "NAVIGATION"
```

## 📊 Topics ROS2 Importants

| Topic | Description | Utilisé pour |
|-------|-------------|--------------|
| `/scan` | LIDAR data | Mapping, Navigation |
| `/cmd_vel` | Commandes de vitesse | Contrôle du robot |
| `/map` | Carte | SLAM, Navigation |
| `/odom` | Odométrie brute | Encodeurs |
| `/odometry/local` | Odométrie fusionnée | EKF (Encodeurs + IMU) |
| `/imu` | Données IMU | Orientation, accélération |
| `/amcl_pose` | Position estimée | Localisation |

## 🛠️ Outils et Commandes Utiles

### Vérification du système
```bash
./check_system.sh                    # Vérification complète
ros2 topic list                      # Voir tous les topics
ros2 node list                       # Voir tous les nodes
ros2 node list | grep nav2           # Vérifier Nav2
```

### Monitoring
```bash
ros2 topic echo /scan                # Voir données LIDAR
ros2 topic echo /map                 # Voir la carte
ros2 topic echo /amcl_pose          # Voir position estimée
ros2 topic echo /odometry/local     # Voir odométrie fusionnée
```

### Dépannage
```bash
killall -9 gzserver gzclient        # Tuer Gazebo si freeze
colcon build && source install/setup.bash  # Rebuild
```

## 🎓 Ressources et Documentation

### Guides du Projet
- **[PROJECT_GUIDE.md](PROJECT_GUIDE.md)** - Guide complet avec tous les détails
- **[QUICK_START.md](QUICK_START.md)** - Commandes rapides
- **[check_system.sh](check_system.sh)** - Script de vérification

### Documentation Technique
- **[docs/IMU_INTEGRATION_GUIDE.md](docs/IMU_INTEGRATION_GUIDE.md)** - Intégration IMU
- **[docs/EKF_TUNING_GUIDE.md](docs/EKF_TUNING_GUIDE.md)** - Tuning EKF
- **[docs/INSTALLATION_GUIDE.md](docs/INSTALLATION_GUIDE.md)** - Installation complète

### Documentation Externe
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Humble](https://docs.ros.org/en/humble/)

## ✅ Checklist de Démarrage

- [ ] ROS2 Humble installé
- [ ] Dépendances installées (slam_toolbox, nav2, etc.)
- [ ] Projet buildé (`colcon build`)
- [ ] Système vérifié (`./check_system.sh`)
- [ ] Carte créée (mapping) OU carte existante dans `maps/`
- [ ] Prêt pour la navigation! 🚀

## 📝 Notes Importantes

⚠️ **Toujours sourcer l'installation**: `source install/setup.bash` dans chaque terminal  
⚠️ **Attendre le démarrage**: Nav2 prend 15-20 secondes pour démarrer complètement  
⚠️ **Mapping lent**: Déplacements lents = meilleure carte  
⚠️ **Pose initiale AMCL**: Essentielle pour la localisation (2D Pose Estimate dans RViz)

## 🤝 Contribution

Pour toute amélioration ou bug report, consulter les logs dans `log/` et la documentation.

## 📄 License

Apache-2.0

---

**Version**: 1.0.0  
**Maintainer**: Louay Mikou  
**Email**: louaymikou17@gmail.com  
**Date**: Janvier 2026

---

**🎉 Prêt à démarrer? Lancez `./check_system.sh` puis consultez [QUICK_START.md](QUICK_START.md)!**

**Terminal 1 - Lancer SLAM Mapping (avec IMU):**

```bash
cd ~/ROS_PROJECT
source install/setup.bash
ros2 launch my_robot_controller slam_mapping.launch.py
```

**Terminal 2 - Contrôler le Robot:**

```bash
cd ~/ROS_PROJECT
source install/setup.bash
python3 src/my_robot_controller/nodes/controllers/keyboard_controller.py
```

Conduisez le robot partout dans l'environnement pour construire une carte complète.

**Terminal 3 - Vérifier la Transformation TF (optionnel):**

```bash
cd ~/ROS_PROJECT
source install/setup.bash
ros2 run tf2_tools view_frames
# Génère frames_<date>.pdf montrant: map -> odom -> base_link
```

**Terminal 4 - Visualiser avec RViz2 (optionnel):**

```bash
cd ~/ROS_PROJECT
source install/setup.bash
rviz2
```

Dans RViz2:
- Fixed Frame: `map`
- Add -> By topic -> `/map`
- Add -> By topic -> `/scan` (LaserScan)
- Add -> TF

**Terminal 5 - Sauvegarder la Carte:**

Une fois la cartographie terminée:

```bash
cd ~/ROS_PROJECT
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

Cela crée 3 fichiers dans `maps/`:
- `my_map.pgm` - Image de la carte
- `my_map.yaml` - Métadonnées de la carte
- `my_map.png` - Aperçu de la carte

---

### ÉTAPE 2: Navigation avec Nav2

**Terminal 1 - Lancer la Simulation:**

```bash
cd ~/ROS_PROJECT
source install/setup.bash
ros2 launch my_robot_controller launch_sim.launch.py
```

**Terminal 2 - Lancer Nav2:**

```bash
cd ~/ROS_PROJECT
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True \
    map:=$HOME/ROS_PROJECT/maps/my_map.yaml \
    params_file:=$HOME/ROS_PROJECT/src/my_robot_controller/config/nav2_params.yaml
```

**Terminal 3 - Lancer RViz2:**

```bash
cd ~/ROS_PROJECT
source install/setup.bash
rviz2
```

Dans RViz2:
1. Fixed Frame: `map`
2. Add -> By topic -> `/map` (Map)
3. Add -> By topic -> `/global_costmap/costmap` (Map)
4. Add -> By topic -> `/local_costmap/costmap` (Map)
5. Add -> TF
6. Toolbar -> **"2D Pose Estimate"** - Cliquez pour définir la position initiale du robot
7. Toolbar -> **"2D Goal Pose"** - Cliquez pour envoyer un objectif de navigation

Le robot naviguera automatiquement vers l'objectif en évitant les obstacles!

---

### ÉTAPE 3: Vérifications et Diagnostics

**Vérifier les Topics:**

```bash
# Liste tous les topics
ros2 topic list

# Vérifier l'odométrie
ros2 topic echo /diff_cont/odom --once

# Vérifier le scan LIDAR
ros2 topic echo /scan --once

# Vérifier la carte
ros2 topic echo /map --once
```

**Vérifier les Transformations TF:**

```bash
# Voir l'arbre TF complet
ros2 run tf2_tools view_frames

# Vérifier une transformation spécifique
ros2 run tf2_ros tf2_echo map base_link
```

**Vérifier Nav2:**

```bash
# Liste des nœuds Nav2
ros2 node list | grep nav2

# Info sur le planificateur
ros2 node info /planner_server
```

---

## 🎮 Modes de Contrôle Rapide

### Simulation Basique
```bash
ros2 launch my_robot_controller launch_sim.launch.py
```

### Avec Contrôle Clavier
```bash
ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py
```

### Avec Manette PS4
```bash
ros2 launch my_robot_controller launch_sim_with_ps4.launch.py
```

## 📖 Documentation

- **[Main Documentation](docs/README.md)** - Complete usage guide
- **[Installation Guide](docs/INSTALLATION_GUIDE.md)** - Setup instructions

## 🎯 Features

- ✅ SLAM mapping with slam_toolbox
- ✅ Teleoperation (keyboard/PS4 controller)
- ✅ Nav2 navigation support
- ✅ ros2_control for robot control
- ✅ Gazebo simulation

---

**Last Updated:** January 2, 2026  
**Branch:** `ikram`
