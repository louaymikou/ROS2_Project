# 📺 Guide d'utilisation de RViz2

## 🎯 Qu'est-ce que RViz2?

RViz2 est l'outil de visualisation 3D de ROS2 qui vous permet de voir:
- Le modèle 3D du robot en temps réel
- Les données du LIDAR (scan laser)
- La carte SLAM construite
- Les trajectoires planifiées (Nav2)
- Les costmaps (obstacles)
- Les transformations TF (frames de coordonnées)

---

## 🚀 Démarrage de RViz2

### Option 1: Avec la simulation complète

RViz2 se lance **automatiquement** avec la mission autonome:

```bash
docker compose up ros2
# ou
ros2 launch my_robot_controller autonomous_mission.launch.py
```

### Option 2: RViz seul (pour se connecter à une simulation existante)

```bash
# Terminal séparé
ros2 launch my_robot_controller rviz.launch.py
```

### Option 3: Désactiver RViz si besoin

```bash
ros2 launch my_robot_controller autonomous_mission.launch.py rviz:=false
```

---

## 🎨 Interface RViz2

### Panneau de gauche: Displays

C'est ici que vous voyez tous les éléments affichés:

✅ **Grid** - Grille de référence au sol  
✅ **RobotModel** - Modèle 3D du robot avec toutes ses articulations  
✅ **TF** - Arbre des transformations (frames)  
✅ **LaserScan** - Points du LIDAR (nuage de points blanc)  
✅ **Map** - Carte SLAM construite (gris = libre, noir = obstacle)  
✅ **GlobalPath** - Chemin global planifié (vert)  
✅ **LocalPath** - Chemin local (rouge)  
✅ **LocalCostmap** - Zone autour du robot avec obstacles détectés  

### Panneau central: Vue 3D

**Contrôles de la caméra:**
- **Clic gauche + glisser** : Rotation de la vue
- **Molette** : Zoom avant/arrière
- **Clic molette + glisser** : Déplacement latéral
- **Shift + clic gauche** : Déplacement de la caméra

### Barre d'outils en haut

- **Interact** : Mode interaction par défaut
- **Move Camera** : Déplacer la caméra
- **Select** : Sélectionner des objets
- **2D Pose Estimate** : Définir la pose initiale du robot
- **2D Goal Pose** : Envoyer un objectif de navigation
- **Publish Point** : Publier un point cliqué

---

## 🔧 Configuration des visualisations

### Activer/Désactiver des éléments

Dans le panneau **Displays**, cochez/décochez les éléments:

```
☑ RobotModel        # Voir le robot 3D
☑ LaserScan         # Voir les données LIDAR
☑ Map               # Voir la carte SLAM
☑ GlobalPath        # Voir le chemin planifié
☐ GlobalCostmap     # Masquer la costmap globale (économie CPU)
☑ LocalCostmap      # Voir les obstacles autour du robot
```

### Modifier les couleurs

1. Cliquez sur un élément dans **Displays**
2. Développez ses propriétés
3. Changez **Color** ou **Alpha** (transparence)

Exemple pour le LIDAR:
```
LaserScan
  ├─ Color: 255; 255; 255 (blanc)
  ├─ Size (m): 0.05 (taille des points)
  └─ Style: Points ou Flat Squares
```

### Changer la vue

Dans le panneau **Views**:
- **Orbit** (par défaut) : Rotation autour d'un point focal
- **Top-down Orthographic** : Vue du dessus (comme une carte)
- **FPS** : Vue première personne

Paramètres de la vue Orbit:
```
Distance: 8.5        # Distance du robot
Pitch: 0.785 (45°)   # Angle vertical
Yaw: 0.785 (45°)     # Angle horizontal
Focal Point: 0,0,0   # Point de focus
```

---

## 📊 Visualisations clés pour le projet

### 1. Vérifier le modèle du robot

**RobotModel** doit montrer:
- ✅ Châssis bleu
- ✅ 4 roues grises
- ✅ Bras orange (2 segments)
- ✅ Pince grise
- ✅ LIDAR rouge sur le dessus

**Problème:** Robot ne s'affiche pas?
```bash
# Vérifier que robot_description est publié
ros2 topic echo /robot_description --once
```

### 2. Vérifier le LIDAR

**LaserScan** doit montrer:
- Nuage de points blancs formant les murs
- 360° de couverture
- Distance max: 15 mètres

**Problème:** Pas de scan?
```bash
# Vérifier les données LIDAR
ros2 topic echo /scan
ros2 topic hz /scan    # Devrait être ~5-10 Hz
```

### 3. Suivre la construction de la carte

**Map** doit montrer:
- Zones grises = espace libre exploré
- Zones noires = obstacles (murs)
- Zones blanches = inexploré

**Astuce:** Activez/désactivez Map pour voir l'évolution

### 4. Observer la navigation

**GlobalPath** (vert):
- Chemin complet de A à B
- Se recalcule si obstacles détectés

**LocalPath** (rouge):
- Segment actuel du chemin
- Mise à jour en temps réel

**LocalCostmap**:
- Rouge foncé = obstacle proche
- Bleu clair = zone libre
- Le robot évite les zones rouges

### 5. Inspecter les transformations TF

**TF** montre l'arbre hiérarchique:
```
map
 └─ odom
     └─ base_link
         ├─ chassis
         │   ├─ arm_1_link
         │   │   └─ arm_2_link
         │   │       └─ gripper_base_link
         │   │           ├─ gripper_left_link
         │   │           └─ gripper_right_link
         │   └─ laser_frame
         ├─ front_left_link
         ├─ front_right_link
         ├─ rear_left_link
         └─ rear_right_link
```

**Frames importantes:**
- `map` : Référence globale (carte)
- `odom` : Odométrie (position intégrée des roues)
- `base_link` : Centre du robot
- `laser_frame` : Position du LIDAR

---

## 🎬 Workflow typique avec RViz2

### Phase 1: Vérification du robot

1. Lancer la simulation avec RViz
2. Vérifier que **RobotModel** s'affiche correctement
3. Observer **TF** pour voir la hiérarchie
4. Vérifier **LaserScan** (points blancs autour du robot)

### Phase 2: Mapping SLAM

1. Activer **Map** et **LaserScan**
2. Déplacer le robot (keyboard ou PS4)
3. Observer la carte se construire en temps réel
4. Zones grises = explorées, noires = murs

### Phase 3: Navigation autonome

1. Activer **GlobalPath** et **LocalPath**
2. Lancer le mission orchestrator
3. Observer:
   - Chemin vert planifié vers Point A
   - Robot suit le chemin rouge
   - Costmap montre les obstacles évités

### Phase 4: Manipulation

1. Zoomer sur le robot
2. Observer **RobotModel** pendant que le bras bouge
3. Voir les joints se déplacer en temps réel
4. Vérifier que la pince s'ouvre/ferme

---

## 🐛 Dépannage RViz2

### RViz ne se lance pas

**Erreur: "cannot open display"**
```bash
# Linux
xhost +local:docker

# WSL2
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
# Assurez-vous que VcXsrv/X410 est lancé
```

**Erreur: Config file not found**
```bash
# Vérifier que le fichier existe
ls install/my_robot_controller/share/my_robot_controller/rviz/robot_view.rviz

# Reconstruire si nécessaire
colcon build --symlink-install
```

### RViz plante ou est lent

**Réduire la charge:**
1. Désactiver **GlobalCostmap** (consomme beaucoup de CPU)
2. Réduire **LaserScan Size (m)** à 0.03
3. Désactiver **TF** si pas nécessaire
4. Passer en vue **Top-down** (plus léger que Orbit)

**Docker:**
```bash
# Vérifier les ressources
docker stats ros2_mobile_manipulator
```

### Éléments manquants

**Pas de robot visible:**
```bash
ros2 topic list | grep robot_description
ros2 run robot_state_publisher robot_state_publisher  # Si manquant
```

**Pas de scan LIDAR:**
```bash
ros2 topic list | grep scan
ros2 topic hz /scan
# Devrait être actif et publier à ~5-10 Hz
```

**Pas de carte:**
```bash
ros2 topic list | grep map
# Vérifier que SLAM Toolbox est lancé
ros2 node list | grep slam
```

**Pas de chemins:**
```bash
ros2 topic list | grep plan
# Vérifier que Nav2 est lancé
ros2 node list | grep nav
```

---

## 💡 Astuces avancées

### Sauvegarder votre configuration RViz

Après avoir ajusté la vue et les displays:
```
File > Save Config As
```

### Vue personnalisée pour la manipulation

Créer une vue proche du robot:
```
Views > Current View
  Distance: 3.0
  Pitch: 1.0 (57°)
  Focal Point: X=0.5, Y=0, Z=0.5
```

### Enregistrer des captures d'écran

RViz supporte les captures via:
```bash
# Installation
sudo apt install ros-humble-rqt-image-view

# Capture
File > Screenshot (dans RViz)
```

### Topics RViz utiles

```bash
# Voir tous les topics disponibles
ros2 topic list

# Topics pour RViz
/robot_description   # Modèle URDF
/joint_states        # État des articulations
/scan                # Données LIDAR
/map                 # Carte SLAM
/odom                # Odométrie
/plan                # Chemin global Nav2
/local_plan          # Chemin local Nav2
/tf                  # Transformations
```

---

## 📖 Ressources supplémentaires

- [Documentation RViz2](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide.html)
- [RViz2 Plugins](https://github.com/ros2/rviz)
- Configuration actuelle: `src/my_robot_controller/rviz/robot_view.rviz`

---

**Bon travail avec RViz2! Vous pouvez maintenant visualiser chaque modification du robot en temps réel! 🚀**
