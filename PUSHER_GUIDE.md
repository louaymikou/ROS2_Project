# 🤖 Système de Poussée de Cubes - Guide d'utilisation

## 📖 Description

Le robot est maintenant équipé d'un mécanisme de poussée qui permet de transporter des cubes depuis l'arrière du robot vers l'avant. Le système comprend :

- **Rail horizontal** : Rail fixé au-dessus du robot dans le sens longitudinal
- **Prisme pousseur** : Mécanisme qui glisse d'arrière vers l'avant pour pousser le cube

## 🎯 Fonctionnement

Le prisme pousseur peut se déplacer **longitudinalement** sur un rail au-dessus du robot :
- **Position arrière** : 0.0 m (position initiale/repos)
- **Position milieu** : 0.3 m (à mi-chemin)
- **Position avant** : 0.6 m (complètement étendu vers l'avant)

Le prisme démarre toujours à l'arrière du robot et peut pousser un cube vers l'avant.

## 🚀 Installation et Build

```bash
cd ~/Ros/ROS2_Project
colcon build --packages-select blue_line_follower
source install/setup.bash
```

## 🎮 Utilisation

### 1. Lancer la simulation

```bash
cd ~/Ros/ROS2_Project
source install/setup.bash
ros2 launch blue_line_follower simulation.launch.py
```

Le système lance automatiquement :
- ✅ Gazebo avec le robot et le mécanisme de poussée
- ✅ Contrôleur de différentiel (roues)
- ✅ Contrôleur du pousseur
- ✅ RViz pour visualisation

### 2. Contrôler le pousseur

Dans un **nouveau terminal** :

```bash
cd ~/Ros/ROS2_Project
source install/setup.bash
ros2 run blue_line_follower pusher_controller
```

Un menu interactif s'affiche :

```
==================================================
🤖 CONTRÔLEUR DE POUSSEUR - Menu
==================================================
1. Pousser vers l'AVANT
2. Position MILIEU
3. Retour à l'ARRIÈRE
4. Séquence complète AVANT
5. Séquence PARTIELLE (milieu)
6. Position personnalisée
0. Quitter
==================================================
```

### 3. Commandes disponibles

#### Option 1 : Pousser vers l'avant
Déplace le prisme complètement vers l'avant (position 0.6m)

#### Option 2 : Position milieu
Déplace le prisme à mi-chemin (position 0.3m)

#### Option 3 : Retour à l'arrière
Ramène le prisme à la position initiale arrière (0.0m)

#### Option 4 : Séquence complète AVANT
Exécute automatiquement :
1. Pousse le cube vers l'avant (0.6m)
2. Attend 1 seconde (le cube glisse/tombe)
3. Retour à l'arrière (0.0m)

#### Option 5 : Séquence partielle
Exécute automatiquement :
1. Pousse le cube à mi-chemin (0.3m)
2. Attend 0.5 seconde
3. Retour à l'arrière (0.0m)

#### Option 6 : Position personnalisée
Permet de spécifier une position exacte entre 0.0 et 0.6 mètres

## 📋 Workflow complet de transport

### Scénario : Transporter un cube du point 0 vers l'avant du robot

1. **Placer le cube à l'arrière du robot** (sous le prisme en position initiale)

2. **Naviguer vers la destination**
   ```bash
   ros2 run blue_line_follower aruco_navigation_client
   # Choisir le marqueur de destination
   ```

3. **Pousser le cube vers l'avant**
   ```bash
   # Terminal 2: Pousseur
   ros2 run blue_line_follower pusher_controller
   # Choisir: 4 (Séquence complète AVANT)
   ```

4. **Le cube est poussé en avant du robot** où il peut être récupéré ou livré

## 🔧 Commandes manuelles (sans menu)

### Envoyer une commande directe avec ros2 topic

```bash
# Pousser à gauche
ros2 action send_goal /pusher_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['pusher_joint'],
    points: [
      {positions: [0.25], velocities: [0.0], time_from_start: {sec: 2, nanosec: 0}}
    ]
  }
}"

# Pousser à droite
ros2 action send_goal /pusher_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectovers l'avant
ros2 action send_goal /pusher_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['pusher_joint'],
    points: [
      {positions: [0.6], velocities: [0.0], time_from_start: {sec: 3, nanosec: 0}}
    ]
  }
}"

# Position milieu
ros2 action send_goal /pusher_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['pusher_joint'],
    points: [
      {positions: [0.3], velocities: [0.0], time_from_start: {sec: 2, nanosec: 0}}
    ]
  }
}"

# Retour à l'arriè
### Lister les contrôleurs actifs
```bash
ros2 control list_controllers
```

Résultat attendu :
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
diff_drive_controller[diff_drive_controller/DiffDriveController] active
pusher_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### Vérifier la position actuelle du pousseur
```bash
ros2 topic echo /joint_states
```

Recherchez `pusher_joint` dans la sortie pour voir sa position actuelle.

### Inspecter les actions disponibles
```bash
ros2 action list
```

Vous devriez voir :
```
/pusher_controller/follow_joint_trajectory
```

## 🛠️ Dépannage

### Le contrôleur du pousseur ne démarre pas

1. Vérifiez que la simulation est lancée :
   ```bash
   ros2 topic list | grep pusher
   ```

2. Vérifiez les contrôleurs :
   ```bash
   ros2 control list_controllers
   ```

3. Relancez le contrôleur manuellement :
   ```bash
   ros2 control load_controller pusher_controller
   ros2 control set_controller_state pusher_controller active
   ```

### Le pousseur ne bouge pas

1. Vérifiez que le serveur d'action est actif :
   ```bash
   ros2 action info /pusher_controller/follow_joint_trajectory
   ```

2. Vérifiez les limites de position dans le URDF (doivent être -0.25 à 0.25)

### Mouvement saccadé

Ajustez la durée du mouvement dans les commandes :
```python
pusher.move_pusher(position, duration_sec=3.0)  # Plus lent = plus fluide
```

## 🎨 Visualisation dans RViz

Dans RViz, vous pouvez visualiser :
- Le robot avec son mécanisme de poussée
- La position actuelle du `pusher_joint`
- Les transformations TF du système

## 📝 Modification des paramètres

### Changer les limites de mouvement

Éditez [urdf/line_follower_robot.urdf.xacro](urdf/line_follower_robot.urdf.xacro) :

```xml
<joint name="pusher_joint" type="prismatic">
  ...
  <limit lower="0.0" upper="0.8" effort="50.0" velocity="0.5"/>
  <!-- Augmentez upper pour plus d'amplitude vers l'avant -->
</joint>
```

### Ajuster la vitesse

Éditez [config/controller_config.yaml](config/controller_config.yaml) :

```yaml
pusher_controller:
  ros__parameters:
    constraints:
      pusher_joint:
        trajectory: 0.1  # Plus grand = plus tolérant
        goal: 0.02       # Précision de l'arrivée
```

## 🤝 Intégration avec le système de navigation

Pour automatiser complètement le processus, vous pouvez créer un script qui combine :

1. Navigation ArUco (marqueur cible)
2. Activation du pousseur
3. Retour au point de départ

Exemple de script d'intégration à créer :

```python
#!/usr/bin/env python3
# Exemple de workflow automatisé
# TODO: À créer pour automatiser le processus complet
```

## 📚 Ressources

- [URDF du robot](urdf/line_follower_robot.urdf.xacro)
- [Configuration des contrôleurs](config/controller_config.yaml)
- [Script de contrôle](blue_line_follower/pusher_controller.py)
- [Fichier de lancement](launch/simulation.launch.py)

## 🎯 Prochaines améliorations possibles

- [ ] Détection automatique de la présence d'un cube
- [ ] Système de préhension pour ramasser les cubes
- [ ] Automatisation complète du cycle de transport
- [ ] Ajout de capteurs de poids/présence
- [ ] Interface graphique pour le contrôle
- [ ] Planification multi-cubes
