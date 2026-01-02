# Guide d'intégration IMU pour SLAM et Nav2

## Configuration effectuée ✅

### 1. Configuration EKF (Extended Kalman Filter)
Le fichier `config/ekf_params.yaml` configure la fusion de capteurs :
- **Odométrie des roues** (`/diff_cont/odom`) : vitesses linéaires et angulaires
- **IMU** (`/imu/data`) : orientation, vitesses angulaires, et accélérations linéaires

### 2. Architecture de fusion
```
[Wheel Encoders] ---> /diff_cont/odom ----\
                                            \
                                             --> [EKF Node] --> /odometry/local --> Nav2
                                            /
[IMU Sensor] -------> /imu/data ----------/
```

### 3. Avantages de la fusion IMU + Wheel Odom
- **Meilleure précision d'orientation** : L'IMU corrige les erreurs de dérive gyroscopique
- **Détection de glissement** : L'IMU détecte quand les roues glissent
- **Réponse plus rapide** : L'IMU fonctionne à 100Hz vs les encodeurs de roues
- **Robustesse** : Si un capteur défaille, l'autre compense

## Comment tester

### 1. Build et source du workspace
```bash
cd /home/wayay/ROS_PROJECT
colcon build --packages-select my_robot_controller
source install/setup.bash
```

### 2. Installer robot_localization (si nécessaire)
```bash
sudo apt install ros-humble-robot-localization
```

### 3. Lancer la simulation avec fusion IMU
```bash
ros2 launch my_robot_controller launch_sim.launch.py
```

### 4. Vérifier les topics
Dans un nouveau terminal :
```bash
# Vérifier que l'IMU publie des données
ros2 topic echo /imu/data

# Vérifier l'odométrie fusionnée
ros2 topic echo /odometry/local

# Vérifier l'odométrie des roues
ros2 topic echo /diff_cont/odom

# Voir tous les TF frames
ros2 run tf2_tools view_frames
```

### 5. Visualiser dans RViz
```bash
rviz2
```
Ajouter :
- **TF** : Pour voir les frames (base_link, odom, imu_link)
- **Imu** : Topic `/imu/data` pour voir l'orientation
- **Odometry** : Topics `/odometry/local` et `/diff_cont/odom` pour comparer

### 6. Lancer SLAM avec l'odométrie fusionnée
```bash
ros2 launch my_robot_controller slam_mapping.launch.py
```

### 7. Lancer Nav2 avec l'odométrie fusionnée
Le Nav2 utilise maintenant `/odometry/local` au lieu de `/diff_cont/odom`.

## Paramètres à ajuster si nécessaire

### Si le robot oscille ou ne suit pas bien le chemin :
Éditez `config/ekf_params.yaml` :
- Augmentez les valeurs de `process_noise_covariance` pour plus de fluidité
- Diminuez-les pour plus de réactivité

### Si l'orientation dérive :
Dans `config/ekf_params.yaml`, section IMU :
- Vérifiez que `imu0_remove_gravitational_acceleration: true`
- Augmentez le poids de l'IMU pour l'orientation (lignes 3-5 de imu0_config)

### Pour voir les performances de l'EKF :
```bash
ros2 topic hz /odometry/local  # Doit être ~50Hz
ros2 run tf2_ros tf2_echo odom base_link  # Voir la transformation
```

## Architecture des fichiers

```
config/
├── ekf_params.yaml          # Configuration de la fusion de capteurs
├── nav2_params.yaml         # Nav2 utilise /odometry/local maintenant
└── my_controllers.yaml

launch/
├── launch_sim.launch.py     # Lance Gazebo + EKF automatiquement
└── robot_localization.launch.py  # Lance uniquement l'EKF (optionnel)

description/
└── imu.xacro                # Description du capteur IMU
```

## Flux de données

1. **Gazebo** simule le robot avec l'IMU
2. **IMU plugin** publie sur `/imu/data`
3. **Diff Drive Controller** publie sur `/diff_cont/odom`
4. **EKF Node** fusionne les deux et publie sur `/odometry/local`
5. **Nav2** utilise `/odometry/local` pour la navigation
6. **SLAM Toolbox** peut utiliser l'odométrie améliorée

## Prochaines étapes recommandées

1. Tester dans différentes conditions (vitesses, rotations rapides)
2. Comparer `/odometry/local` vs `/diff_cont/odom` pour voir l'amélioration
3. Ajuster les paramètres de covariance selon vos besoins
4. Éventuellement ajouter un deuxième EKF pour la fusion map->odom si nécessaire

## Troubleshooting

**Problème** : Le nœud EKF ne démarre pas
```bash
# Vérifier les dépendances
ros2 pkg list | grep robot_localization
# Si absent, installer :
sudo apt install ros-humble-robot-localization
```

**Problème** : Pas de données IMU
```bash
# Vérifier que l'IMU est bien dans le URDF
ros2 param get /robot_state_publisher robot_description | grep imu_link
```

**Problème** : L'odométrie fusionnée n'est pas publiée
```bash
# Vérifier les logs de l'EKF
ros2 node info /ekf_filter_node
```
