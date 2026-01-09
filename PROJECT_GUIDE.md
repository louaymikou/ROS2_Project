# 🤖 ROS2 Mobile Manipulator - Guide Complet de Navigation et Mapping

## 📋 Table des Matières
1. [Vue d'ensemble du Projet](#vue-densemble-du-projet)
2. [Prérequis et Installation](#prérequis-et-installation)
3. [Mapping avec SLAM](#mapping-avec-slam)
4. [Navigation Autonome avec Nav2](#navigation-autonome-avec-nav2)
5. [Dépannage](#dépannage)
6. [Architecture du Projet](#architecture-du-projet)

---

## 🎯 Vue d'ensemble du Projet

Ce projet ROS2 permet de:
- **Mapper** un environnement avec SLAM Toolbox
- **Naviguer** de manière autonome avec Nav2
- **Contrôler** un robot mobile avec bras manipulateur et pince

### Composants Principaux
- **Robot**: Base mobile différentielle + Bras à 3 joints + Pince
- **Capteurs**: LIDAR pour le mapping et la navigation
- **Simulation**: Gazebo avec monde personnalisé
- **Navigation**: SLAM Toolbox (mapping) + Nav2 (navigation autonome)

---

## 🔧 Prérequis et Installation

### Dépendances ROS2
Assurez-vous que les packages suivants sont installés:

```bash
# Packages Navigation et SLAM
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-nav2-map-server \
                 ros-humble-robot-localization \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-publisher \
                 ros-humble-robot-state-publisher

# Outils supplémentaires
sudo apt install ros-humble-rviz2
```

### Structure du Projet
```
ROS2_Project/
├── src/
│   └── my_robot_controller/
│       ├── config/          # Fichiers de configuration
│       │   ├── nav2_params.yaml
│       │   └── slam_params.yaml
│       ├── description/     # URDF/Xacro du robot
│       ├── launch/          # Fichiers de lancement
│       │   ├── slam_mapping.launch.py
│       │   └── navigation.launch.py
│       ├── maps/            # Cartes sauvegardées
│       │   ├── my_robot_map.yaml
│       │   └── my_robot_map.pgm
│       ├── models/          # Modèles 3D (STL)
│       ├── nodes/           # Scripts Python
│       │   ├── controllers/
│       │   │   └── keyboard_controller.py
│       │   └── navigation/
│       │       └── auto_navigator.py
│       └── worlds/          # Mondes Gazebo
├── build/
├── install/
└── log/
```

---

## 🗺️ Mapping avec SLAM

### Objectif
Créer une carte de l'environnement en déplaçant le robot manuellement tout en scannant avec le LIDAR.

### Procédure Complète

#### **Terminal 1**: Lancement de Gazebo et SLAM
```bash
cd ~/ROS2_Project
colcon build
source install/setup.bash
ros2 launch my_robot_controller slam_mapping.launch.py
```

**Ce qui se lance:**
- ✅ Gazebo avec le monde simulé
- ✅ Robot state publisher
- ✅ Spawn du robot (après 3s)
- ✅ Controllers (diff_drive, joint_state, arm, gripper) (après 5s)
- ✅ SLAM Toolbox (après 7s)

**Attendez** ~10 secondes que tous les composants démarrent.

---

#### **Terminal 2**: RViz pour visualiser le mapping
```bash
cd ~/ROS2_Project
source install/setup.bash
rviz2
```

**Configuration RViz pour le mapping:**
1. **Fixed Frame**: `map`
2. **Ajouter** les affichages suivants:
   - **RobotModel**: Pour voir le robot
   - **LaserScan**: Topic `/scan` pour voir le LIDAR
   - **Map**: Topic `/map` pour voir la carte en construction
   - **TF**: Pour voir les transformations

**Optionnel**: Sauvegarder cette configuration RViz pour la réutiliser.

---

#### **Terminal 3**: Contrôle par clavier
```bash
cd ~/ROS2_Project
source install/setup.bash
python3 src/my_robot_controller/nodes/controllers/keyboard_controller.py
```

**Commandes disponibles:**
```
╔══════════════════════════════════════════════════════════╗
║     CONTRÔLE DU ROBOT                                    ║
╠══════════════════════════════════════════════════════════╣
║ MOUVEMENT:                       VITESSES:               ║
║   I : Avancer                      1 : Lent (0.2 m/s)    ║
║   K : Reculer                      2 : Moyen (0.5)       ║
║   J : Tourner à gauche             3 : Rapide (1.0)      ║
║   L : Tourner à droite                                   ║
║   ESPACE : Arrêter                                       ║
╠══════════════════════════════════════════════════════════╣
║ BRAS (Incrémental):              PINCE:                  ║
║   A/Q : Épaule Haut/Bas            O : Ouvrir           ║
║   Z/S : Coude Ext/Int              P : Fermer           ║
║   R/F : Rotation G/D               0 : Position Home     ║
╠══════════════════════════════════════════════════════════╣
║ QUITTER: W ou Ctrl+C                                     ║
╚══════════════════════════════════════════════════════════╝
```

**Conseils pour un bon mapping:**
1. Déplacez-vous **lentement** (vitesse 1 ou 2)
2. Faites des **rotations complètes** aux carrefours
3. **Couvrez tout l'environnement** méthodiquement
4. **Évitez les mouvements brusques**
5. Surveillez RViz pour vérifier la qualité de la carte

---

#### **Terminal 4**: Sauvegarde de la carte (après mapping complet)
```bash
cd ~/ROS2_Project
source install/setup.bash

# Sauvegarder dans le home directory
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map

# OU sauvegarder directement dans le package (recommandé)
ros2 run nav2_map_server map_saver_cli -f src/my_robot_controller/maps/my_robot_map
```

**Fichiers générés:**
- `my_robot_map.yaml` - Métadonnées de la carte
- `my_robot_map.pgm` - Image de la carte

**Vérification:**
```bash
# Vérifier que les fichiers existent
ls -lh src/my_robot_controller/maps/
```

---

### ✅ Checklist Mapping
- [ ] Gazebo lancé avec le monde
- [ ] Robot spawné correctement
- [ ] SLAM Toolbox actif (vérifier logs)
- [ ] RViz affiche le topic `/map`
- [ ] Contrôle clavier fonctionne
- [ ] Carte complète et cohérente
- [ ] Carte sauvegardée dans `maps/`

---

## 🧭 Navigation Autonome avec Nav2

### Objectif
Utiliser la carte créée pour naviguer de manière autonome vers des points de destination.

### Prérequis
⚠️ **Vous devez avoir une carte sauvegardée** (`my_robot_map.yaml` et `my_robot_map.pgm`) dans le dossier `src/my_robot_controller/maps/`

### Procédure Complète

#### **Terminal 1**: Lancement de Gazebo et Nav2
```bash
cd ~/ROS2_Project
colcon build --packages-select my_robot_controller
source install/setup.bash
ros2 launch my_robot_controller navigation.launch.py
```

**Ce qui se lance:**
- ✅ Gazebo avec le monde simulé
- ✅ Robot state publisher
- ✅ Spawn du robot (après 3s)
- ✅ Controllers (après 5s)
- ✅ Map Server (après 8s)
- ✅ AMCL Localisation (après 10s)
- ✅ Nav2 Stack complet (après 12s)

**Attendez** ~15-20 secondes que tous les composants Nav2 démarrent.

**Vérification:**
```bash
# Dans un autre terminal
ros2 node list | grep nav2
```
Vous devriez voir plusieurs nœuds Nav2 (controller, planner, bt_navigator, etc.)

---

#### **Terminal 2**: Script de navigation autonome
```bash
cd ~/ROS2_Project
source install/setup.bash
ros2 run my_robot_controller auto_navigator.py demo
```

**Modes disponibles:**
- `demo` - Navigation vers des points prédéfinis
- `patrol` - Patrouille entre plusieurs points
- `interactive` - Mode interactif pour définir des points

**Fonctionnalités du script:**
- ✅ Définir la pose initiale (AMCL)
- ✅ Naviguer vers des destinations
- ✅ Suivre le statut de navigation
- ✅ Gérer les échecs et réessayer

---

#### **Terminal 3**: RViz avec configuration Nav2
```bash
cd ~/ROS2_Project
source install/setup.bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Configuration RViz pour la navigation:**

Cette configuration RViz officielle Nav2 affiche:
- 🗺️ **Map** - La carte statique
- 📍 **Global Costmap** - Obstacles globaux
- 📍 **Local Costmap** - Obstacles locaux
- 🎯 **Global Plan** - Trajectoire planifiée (bleue)
- 🎯 **Local Plan** - Trajectoire locale (rouge)
- 🤖 **Robot Model** - Le robot
- 🔴 **Particle Cloud** - Particules AMCL (localisation)

**Actions manuelles dans RViz:**
1. **2D Pose Estimate**: Définir la position initiale du robot
   - Cliquez sur le bouton
   - Cliquez sur la carte où se trouve le robot
   - Glissez pour définir l'orientation

2. **Nav2 Goal**: Envoyer un objectif de navigation
   - Cliquez sur le bouton "Nav2 Goal"
   - Cliquez sur la destination
   - Glissez pour définir l'orientation finale

---

### ✅ Checklist Navigation
- [ ] Gazebo lancé avec le monde
- [ ] Robot spawné correctement
- [ ] Map Server charge la carte (vérifier logs)
- [ ] AMCL actif (particules visibles dans RViz)
- [ ] Nav2 nodes actifs (`ros2 node list`)
- [ ] RViz affiche costmaps et plans
- [ ] Pose initiale définie (2D Pose Estimate)
- [ ] Navigation vers objectif fonctionne

---

## 🔍 Dépannage

### Problème: "Map file not found"
**Solution:**
```bash
# Vérifier que la carte existe
ls -lh src/my_robot_controller/maps/

# Si absent, refaire le mapping et sauvegarder
```

### Problème: "AMCL ne localise pas le robot"
**Solution:**
1. Définir manuellement la pose initiale dans RViz (2D Pose Estimate)
2. Vérifier que la carte chargée correspond à l'environnement Gazebo
3. Déplacer légèrement le robot pour que AMCL converge

### Problème: "Nav2 ne planifie pas de chemin"
**Solution:**
1. Vérifier que la carte est bien chargée (`ros2 topic echo /map`)
2. Vérifier les costmaps dans RViz
3. S'assurer que la destination est accessible (pas dans un obstacle)
4. Vérifier les paramètres Nav2 dans `config/nav2_params.yaml`

### Problème: "Robot bloqué ou rotation sur place"
**Solution:**
1. Réduire la vitesse dans `nav2_params.yaml`
2. Ajuster les paramètres de planification locale
3. Vérifier que les costmaps ne sont pas trop restrictives
4. Rebuild et relancer: `colcon build && source install/setup.bash`

### Problème: "Controllers non trouvés"
**Solution:**
```bash
# Vérifier que les controllers sont chargés
ros2 control list_controllers

# Relancer si nécessaire
ros2 control load_controller diff_cont
ros2 control load_controller joint_broad
```

### Problème: "Gazebo crash ou freeze"
**Solution:**
```bash
# Tuer tous les processus Gazebo
killall -9 gzserver gzclient

# Relancer proprement
ros2 launch my_robot_controller slam_mapping.launch.py
# OU
ros2 launch my_robot_controller navigation.launch.py
```

### Logs Utiles
```bash
# Voir les logs ROS2
ros2 topic list
ros2 topic echo /map
ros2 node list
ros2 node info /slam_toolbox
ros2 node info /amcl

# Logs de build
cat log/latest_build/events.log
```

---

## 📐 Architecture du Projet

### Fichiers de Configuration

#### `config/slam_params.yaml`
Paramètres pour SLAM Toolbox:
- Mode: `mapping` (asynchrone)
- Résolution de la carte
- Paramètres de scan matching
- Use sim time: true

#### `config/nav2_params.yaml`
Paramètres pour Nav2:
- **Controller**: Suivi de trajectoire (DWB/TEB)
- **Planner**: Planification globale (NavFn/Smac)
- **Costmaps**: Obstacles statiques/dynamiques
- **Behavior Server**: Comportements (spin, backup, wait)
- **AMCL**: Localisation par filtrage particulaire

### Fichiers de Launch

#### `slam_mapping.launch.py`
Séquence de démarrage:
1. Gazebo + World
2. Robot State Publisher
3. Spawn Robot (3s)
4. Controllers (5s)
5. SLAM Toolbox (7s)

#### `navigation.launch.py`
Séquence de démarrage:
1. Gazebo + World
2. Robot State Publisher
3. Spawn Robot (3s)
4. Controllers (5s)
5. Map Server + Lifecycle (8s)
6. AMCL + Lifecycle (10s)
7. Nav2 Bringup (12s)

### Scripts Python

#### `keyboard_controller.py`
- Contrôle téléopéré du robot
- Base mobile: I/K/J/L
- Bras: A/Q, Z/S, R/F
- Pince: O/P
- Modes de vitesse: 1/2/3

#### `auto_navigator.py`
- Client d'action Nav2
- Définition de pose initiale
- Navigation vers objectifs
- Modes: demo, patrol, interactive

---

## 🎓 Workflow Complet Recommandé

### Phase 1: Mapping (Première fois)
1. ✅ Lancer SLAM + Gazebo (Terminal 1)
2. ✅ Ouvrir RViz (Terminal 2)
3. ✅ Contrôler le robot (Terminal 3)
4. ✅ Explorer tout l'environnement
5. ✅ Sauvegarder la carte (Terminal 4)

### Phase 2: Navigation (Utilisations suivantes)
1. ✅ Lancer Nav2 + Gazebo (Terminal 1)
2. ✅ Lancer le navigateur autonome (Terminal 2)
3. ✅ Ouvrir RViz Nav2 (Terminal 3)
4. ✅ Définir pose initiale dans RViz
5. ✅ Envoyer des objectifs de navigation

### Phase 3: Amélioration Continue
- Ajuster les paramètres SLAM si la carte n'est pas nette
- Tuner Nav2 pour améliorer la navigation
- Créer de nouveaux scripts de navigation
- Ajouter de nouvelles fonctionnalités (détection d'objets, etc.)

---

## 📚 Ressources Supplémentaires

### Documentation Officielle
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Humble](https://docs.ros.org/en/humble/)

### Fichiers du Projet à Explorer
- [EKF Tuning Guide](docs/EKF_TUNING_GUIDE.md)
- [IMU Integration Guide](docs/IMU_INTEGRATION_GUIDE.md)
- [Installation Guide](docs/INSTALLATION_GUIDE.md)
- [Useful Commands](USEFUL_COMMANDS.md)

---

## 🚀 Commandes Rapides

### Build et Source
```bash
cd ~/ROS2_Project
colcon build
source install/setup.bash
```

### Mapping Rapide
```bash
# T1
ros2 launch my_robot_controller slam_mapping.launch.py

# T2
rviz2

# T3
python3 src/my_robot_controller/nodes/controllers/keyboard_controller.py

# T4 (après mapping)
ros2 run nav2_map_server map_saver_cli -f src/my_robot_controller/maps/my_robot_map
```

### Navigation Rapide
```bash
# T1
ros2 launch my_robot_controller navigation.launch.py

# T2
ros2 run my_robot_controller auto_navigator.py demo

# T3
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

---

## ✨ Fonctionnalités Avancées

### Navigation vers des Points Personnalisés
Modifier `auto_navigator.py` pour ajouter vos propres destinations:
```python
# Exemple de destinations
destinations = [
    {"x": 2.0, "y": 3.0, "yaw": 0.0},
    {"x": -1.0, "y": 2.0, "yaw": 1.57},
    {"x": 0.0, "y": 0.0, "yaw": 0.0}
]
```

### Sauvegarde de Configurations RViz
```bash
# Dans RViz: File > Save Config As
# Sauvegarder dans: src/my_robot_controller/rviz/my_config.rviz

# Charger ensuite:
rviz2 -d src/my_robot_controller/rviz/my_config.rviz
```

### Monitoring en Temps Réel
```bash
# Voir la position estimée (AMCL)
ros2 topic echo /amcl_pose

# Voir le statut de navigation
ros2 topic echo /navigate_to_pose/_action/status

# Voir les commandes de vitesse
ros2 topic echo /cmd_vel
```

---

## 📝 Notes Importantes

1. **Toujours sourcer après build**: `source install/setup.bash`
2. **Use sim time**: Tous les nœuds utilisent `use_sim_time: True`
3. **Séquence de démarrage**: Respecter les délais entre composants
4. **Qualité de la carte**: Détermine la qualité de la navigation
5. **Pose initiale**: Cruciale pour une bonne localisation AMCL

---

**Version**: 1.0.0  
**Date**: Janvier 2026  
**Auteur**: Louay Mikou  
**License**: Apache-2.0

---

## 🎉 Félicitations!

Vous êtes maintenant prêt à utiliser votre robot mobile pour le mapping et la navigation autonome! 🚀

Pour toute question, consultez les fichiers de documentation dans `docs/` ou les logs dans `log/`.

**Bon mapping et bonne navigation! 🗺️🤖**
