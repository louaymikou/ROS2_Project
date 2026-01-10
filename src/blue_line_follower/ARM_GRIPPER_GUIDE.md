# Contrôle du Bras et du Gripper

## Description

Le robot dispose maintenant d'un bras robotique à 3 degrés de liberté (épaule, coude, rotation) et d'un gripper à 2 doigts.

## Architecture

### Joints du Bras
- **shoulder_joint**: Articulation de l'épaule (rotation Y, limites: -1.57 à 1.57 rad)
- **elbow_joint**: Articulation du coude (rotation Y, limites: -2.5 à 2.5 rad)
- **gripper_rotate_joint**: Rotation de la pince (rotation Z, limites: -3.14 à 3.14 rad)

### Joints du Gripper
- **gripper_left_joint**: Doigt gauche (prismatique, limites: -0.60 à 0.0 m)
- **gripper_right_joint**: Doigt droit (prismatique, limites: -0.60 à 0.0 m)

## Contrôleurs ROS2 Control

### arm_controller
- **Type**: joint_trajectory_controller/JointTrajectoryController
- **Topic**: `/arm_controller/joint_trajectory`
- **Joints contrôlés**: shoulder_joint, elbow_joint, gripper_rotate_joint
- **Mode**: Position

### gripper_controller
- **Type**: joint_trajectory_controller/JointTrajectoryController
- **Topic**: `/gripper_controller/joint_trajectory`
- **Joints contrôlés**: gripper_left_joint, gripper_right_joint
- **Mode**: Position

## Lancement de la Simulation

```bash
# Source l'environnement
source install/setup.bash

# Lancer la simulation complète (robot + bras + gripper)
ros2 launch blue_line_follower simulation.launch.py
```

La simulation lance automatiquement tous les contrôleurs nécessaires :
- `joint_state_broadcaster` : Publie l'état de tous les joints
- `diff_drive_controller` : Contrôle les roues
- `arm_controller` : Contrôle le bras
- `gripper_controller` : Contrôle le gripper

## Utilisation du Commander

### Lancement du Commander
```bash
ros2 run blue_line_follower arm_commander
```

### Commandes Clavier

- **e** : Étendre le bras (position de travail)
  - Épaule: 0.75 rad
  - Coude: 0.75 rad
  - Rotation: conservée

- **r** : Ranger le bras (position repliée)
  - Épaule: -1.0 rad
  - Coude: 3.14 rad
  - Rotation: conservée

- **g** : Ouvrir le gripper
  - Les deux doigts: 0.0 (ouvert)

- **c** : Fermer le gripper (Close)
  - Les deux doigts: -0.5 (fermé partiellement)

- **m** : Mode manuel
  - Permet de choisir un moteur spécifique et un angle
  - Moteurs disponibles:
    - 1 = Épaule
    - 2 = Coude
    - 3 = Rotation de la pince

- **q** : Quitter le programme

### Exemple de Session
```
CONTROLE DU BRAS AMELIORE
---------------------------
Touches :
   e : Etendre (Position travail)
   r : Ranger (Position repliée)
   m : Mode MANUEL (Choisir moteur et angle)
   g : Gripper OUVERT
   c : Gripper FERME
   q : Quitter

# Appuyez sur 'e' pour étendre le bras
[INFO] Commande bras -> Epaule: 0.75, Coude: 0.75, Rotation: 0.00

# Appuyez sur 'g' pour ouvrir le gripper
[INFO] Commande gripper -> Gauche: 0.00, Droite: 0.00

# Appuyez sur 'm' pour le mode manuel
--- MODE MANUEL ---
Quel moteur ? (1=Epaule, 2=Coude, 3=Rotation) : 3
Quel angle ? (ex: 1.57 pour 90°) : 1.57
[INFO] Rotation de la pince vers 1.57
[INFO] Commande bras -> Epaule: 0.75, Coude: 0.75, Rotation: 1.57

# Appuyez sur 'c' pour fermer le gripper
[INFO] Commande gripper -> Gauche: -0.50, Droite: -0.50
```

## Contrôle Manuel via ROS2 Topics

### Contrôle du Bras
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
points:
- positions: [0.5, 1.0, 0.0]
  time_from_start: {sec: 1, nanosec: 0}
"
```

### Contrôle du Gripper
```bash
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['gripper_left_joint', 'gripper_right_joint']
points:
- positions: [-0.3, -0.3]
  time_from_start: {sec: 1, nanosec: 0}
"
```

## Visualisation dans RViz

RViz se lance automatiquement avec la simulation et affiche :
- Le modèle du robot avec le bras
- Les transformations TF
- L'état des joints

Pour visualiser les joints du bras :
1. Ajoutez un plugin "RobotModel"
2. Ajoutez un plugin "TF" pour voir les frames
3. Ajoutez "JointStatePublisher GUI" pour contrôler manuellement (optionnel)

## Topics Disponibles

```bash
# Voir tous les topics
ros2 topic list

# Topics principaux:
/arm_controller/joint_trajectory          # Commande du bras
/gripper_controller/joint_trajectory      # Commande du gripper
/joint_states                             # État de tous les joints
/arm_controller/state                     # État du contrôleur du bras
/gripper_controller/state                 # État du contrôleur du gripper
```

## Dépannage

### Le bras ne bouge pas
1. Vérifiez que les contrôleurs sont actifs:
```bash
ros2 control list_controllers
```

Vous devriez voir:
```
arm_controller[joint_trajectory_controller/JointTrajectoryController] active
gripper_controller[joint_trajectory_controller/JointTrajectoryController] active
```

2. Si un contrôleur est inactif, relancez-le:
```bash
ros2 control load_controller arm_controller
ros2 control set_controller_state arm_controller start
```

### Le gripper ne se ferme pas complètement
- Les limites sont configurées à -0.60 m maximum
- Ajustez la valeur dans la commande si nécessaire

### Collisions avec le châssis
- Les positions de repliage et d'extension sont optimisées pour éviter les collisions
- En mode manuel, faites attention aux limites physiques

## Intégration avec la Navigation ArUco

Le bras peut être utilisé conjointement avec le système de navigation ArUco. Par exemple:
1. Le robot navigue vers un ArUco marker
2. Le bras s'étend pour saisir un objet
3. Le gripper se ferme
4. Le robot retourne à la base avec l'objet

## Fichiers de Configuration

- **URDF**: `src/blue_line_follower/urdf/line_follower_robot.urdf.xacro`
- **Contrôleurs**: `src/blue_line_follower/config/controller_config.yaml`
- **Node Commander**: `src/blue_line_follower/blue_line_follower/arm_commander.py`
- **Launch File**: `src/blue_line_follower/launch/simulation.launch.py`
