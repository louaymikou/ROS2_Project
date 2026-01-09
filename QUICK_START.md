# 🚀 Quick Start Guide - ROS2 Mobile Manipulator

## 📋 Commandes Rapides

### 🗺️ MAPPING (Première utilisation)

#### Terminal 1 - Lancement Gazebo + SLAM
```bash
cd ~/ROS2_Project
colcon build
source install/setup.bash
ros2 launch my_robot_controller slam_mapping.launch.py
```
⏳ **Attendre 10 secondes** pour le démarrage complet

---

#### Terminal 2 - Visualisation RViz
```bash
cd ~/ROS2_Project
source install/setup.bash
rviz2
```
**Configuration RViz:**
- Fixed Frame: `map`
- Ajouter: RobotModel, LaserScan (/scan), Map (/map), TF

---

#### Terminal 3 - Contrôle Clavier
```bash
cd ~/ROS2_Project
source install/setup.bash
python3 src/my_robot_controller/nodes/controllers/keyboard_controller.py
```

**Commandes:**
- `I` = Avancer | `K` = Reculer | `J` = Gauche | `L` = Droite
- `1` = Lent | `2` = Moyen | `3` = Rapide
- `ESPACE` = Stop | `W` = Quitter

**💡 Conseil**: Déplacez-vous lentement (mode 1 ou 2) pour un bon mapping

---

#### Terminal 4 - Sauvegarder la Carte (après mapping)
```bash
cd ~/ROS2_Project
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/my_robot_controller/maps/my_robot_map
```

**Vérification:**
```bash
ls -lh src/my_robot_controller/maps/
# Devrait afficher: my_robot_map.yaml et my_robot_map.pgm
```

---

## 🧭 NAVIGATION (Après avoir créé la carte)

#### Terminal 1 - Lancement Gazebo + Nav2
```bash
cd ~/ROS2_Project
colcon build --packages-select my_robot_controller
source install/setup.bash
ros2 launch my_robot_controller navigation.launch.py
```
⏳ **Attendre 15-20 secondes** pour le démarrage Nav2

---

#### Terminal 2 - Navigation Autonome
```bash
cd ~/ROS2_Project
source install/setup.bash
ros2 run my_robot_controller auto_navigator.py demo
```

**Modes disponibles:**
- `demo` - Points de démo
- `patrol` - Patrouille
- `interactive` - Mode interactif

---

#### Terminal 3 - RViz Nav2
```bash
cd ~/ROS2_Project
source install/setup.bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Actions dans RViz:**
1. **2D Pose Estimate** → Cliquer sur position initiale du robot
2. **Nav2 Goal** → Cliquer sur destination

---

## ✅ Checklist de Vérification

### Avant le Mapping:
- [ ] ROS2 Humble installé
- [ ] Packages Nav2 et SLAM Toolbox installés
- [ ] Workspace build: `colcon build`

### Pendant le Mapping:
- [ ] Gazebo affiche le robot et le monde
- [ ] RViz affiche le topic `/map`
- [ ] Contrôle clavier fonctionne
- [ ] La carte se construit dans RViz

### Après le Mapping:
- [ ] Carte sauvegardée dans `maps/`
- [ ] Fichiers `.yaml` et `.pgm` présents

### Pendant la Navigation:
- [ ] Map Server charge la carte (logs)
- [ ] AMCL affiche les particules (rouge dans RViz)
- [ ] Nav2 nodes actifs: `ros2 node list | grep nav2`
- [ ] Costmaps visibles dans RViz
- [ ] Navigation vers objectif fonctionne

---

## 🔧 Dépannage Rapide

### Carte non trouvée
```bash
ls src/my_robot_controller/maps/
# Si vide → Refaire le mapping
```

### Nav2 ne démarre pas
```bash
ros2 node list | grep nav2
# Si vide → Attendre plus longtemps (20s)
```

### Gazebo freeze
```bash
killall -9 gzserver gzclient
# Puis relancer
```

### AMCL ne localise pas
```bash
# Dans RViz: Utiliser "2D Pose Estimate"
# Cliquer sur la position approximative du robot
```

---

## 📊 Vérification des Topics

```bash
# Vérifier que tout fonctionne
ros2 topic list

# Topics importants:
# /scan          - LIDAR
# /map           - Carte (mapping/navigation)
# /cmd_vel       - Commandes de vitesse
# /tf            - Transformations
# /amcl_pose     - Position estimée (navigation)
```

---

## 🎯 Workflow Typique

### Session 1: Créer la Carte
1. Lancer SLAM (T1)
2. Ouvrir RViz (T2)
3. Piloter le robot (T3)
4. Explorer tout l'environnement
5. Sauvegarder la carte (T4)

### Session 2+: Navigation
1. Lancer Nav2 (T1)
2. Lancer navigateur (T2)
3. Ouvrir RViz Nav2 (T3)
4. Définir pose initiale
5. Naviguer!

---

## 📝 Notes Importantes

⚠️ **Toujours sourcer**: `source install/setup.bash` dans chaque terminal  
⚠️ **Attendre le démarrage**: Nav2 prend ~15-20 secondes  
⚠️ **Mapping lent**: Déplacez-vous lentement pour une bonne carte  
⚠️ **Pose initiale**: Essentielle pour AMCL (2D Pose Estimate dans RViz)

---

## 🎓 Pour Aller Plus Loin

📖 Guide complet: [PROJECT_GUIDE.md](PROJECT_GUIDE.md)  
📖 Documentation Nav2: https://navigation.ros.org/  
📖 SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

---

**Bon mapping et bonne navigation! 🗺️🤖**
