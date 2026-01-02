# ROS2 Mobile Robot Project

🤖 Mobile robot with teleoperation, SLAM mapping, and navigation capabilities.

## 📁 Project Structure

```
ROS_PROJECT/
├── docs/                    # Documentation
│   ├── README.md           # Main project documentation
│   └── INSTALLATION_GUIDE.md
├── maps/                    # Generated maps
│   ├── my_robot_map.pgm
│   ├── my_robot_map.png
│   └── my_robot_map.yaml
├── config/                  # Configuration files
│   ├── slam_params.yaml
│   └── nav2_params/
├── src/                     # Source code
│   └── my_robot_controller/
└── .gitignore

```

## 🚀 Quick Start

See [docs/README.md](docs/README.md) for complete documentation.

### Installation

```bash
# Build workspace
cd ~/ROS_PROJECT
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 📍 Complete Workflow: SLAM Mapping + Nav2 Navigation

### ÉTAPE 1: Créer une Carte avec SLAM

**Terminal 1 - Lancer SLAM Mapping:**

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
