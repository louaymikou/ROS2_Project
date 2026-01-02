# Architecture de Fusion de Capteurs IMU

## Flux de Données

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          GAZEBO SIMULATION                               │
│  ┌──────────────┐      ┌─────────────┐      ┌──────────────┐          │
│  │ Wheel Sensors│      │ IMU Sensor  │      │ Lidar Sensor │          │
│  │  (Encoders)  │      │  (MPU-6050) │      │  (RPLidar)   │          │
│  └──────┬───────┘      └──────┬──────┘      └──────┬───────┘          │
└─────────┼─────────────────────┼─────────────────────┼──────────────────┘
          │                     │                     │
          │ Linear/Angular      │ Orientation,        │ LaserScan
          │ Velocities          │ Angular Vel,        │ Data
          │                     │ Linear Accel        │
          ↓                     ↓                     ↓
    /diff_cont/odom        /imu/data               /scan
    nav_msgs/Odometry      sensor_msgs/Imu         sensor_msgs/LaserScan
          │                     │                     │
          │                     │                     │
          ├─────────────────────┤                     │
          │                     │                     │
          ↓                     ↓                     │
    ┌─────────────────────────────────────┐          │
    │   ROBOT_LOCALIZATION (EKF Node)     │          │
    │                                     │          │
    │  Extended Kalman Filter             │          │
    │  - Fusionne odom + IMU              │          │
    │  - Corrige les dérives              │          │
    │  - Détecte les glissements          │          │
    │  - Publie TF: odom → base_link     │          │
    └────────────┬────────────────────────┘          │
                 │                                    │
                 │ Fused Odometry                     │
                 ↓                                    │
          /odometry/local                             │
          nav_msgs/Odometry                           │
                 │                                    │
                 ├────────────────────────────────────┤
                 │                                    │
                 ↓                                    ↓
    ┌─────────────────────────────────────────────────────────┐
    │              SLAM TOOLBOX / NAV2                        │
    │                                                         │
    │  ┌─────────────────┐        ┌──────────────────┐      │
    │  │  SLAM Toolbox   │        │  Nav2 Stack      │      │
    │  │  - Mapping      │        │  - Localization  │      │
    │  │  - Loop Closure │        │  - Path Planning │      │
    │  │                 │        │  - Control       │      │
    │  └─────────────────┘        └──────────────────┘      │
    └─────────────────────────────────────────────────────────┘
```

## Comparaison: Avant vs Après IMU

### AVANT (Wheel Odometry uniquement)
```
┌────────────┐
│  Encoders  │ → /diff_cont/odom → SLAM/Nav2
└────────────┘
❌ Problèmes:
   - Dérive en rotation
   - Erreurs sur surfaces glissantes
   - Pas de correction d'orientation
```

### APRÈS (Wheel Odometry + IMU)
```
┌────────────┐
│  Encoders  │ ──┐
└────────────┘   │
                 ├→ [EKF] → /odometry/local → SLAM/Nav2
┌────────────┐   │
│    IMU     │ ──┘
└────────────┘
✅ Avantages:
   - Orientation précise
   - Détection de glissement
   - Réponse plus rapide
   - Robustesse accrue
```

## Configuration EKF

### Sources de Données

| Source          | Topic            | Données Utilisées                          |
|-----------------|------------------|--------------------------------------------|
| Wheel Encoders  | /diff_cont/odom  | ✅ Linear velocity (x, y)                  |
|                 |                  | ✅ Angular velocity (yaw)                  |
|                 |                  | ❌ Position (non utilisée)                 |
| IMU             | /imu/data        | ✅ Orientation (roll, pitch, yaw)          |
|                 |                  | ✅ Angular velocity (roll, pitch, yaw)     |
|                 |                  | ✅ Linear acceleration (x, y, z)           |

### Configuration Matrix (15 états)

```
État          Position  Orientation  Velocity     Angular Vel  Acceleration
              x  y  z   r  p  y      ẋ  ẏ  ż      ṙ  ṗ  ẏ      ẍ  ÿ  z̈
odom0 (wheels)F  F  F   F  F  F      T  T  F      F  F  T      F  F  F
imu0  (IMU)   F  F  F   T  T  T      F  F  F      T  T  T      T  T  T

F = False (non utilisé)
T = True (utilisé)
```

## Frames TF

```
        map
         │
         │ (fourni par AMCL/SLAM)
         ↓
        odom ←─────────────────┐
         │                     │
         │ (fourni par EKF)    │ Publie TF
         ↓                     │
      base_link                │
         ├───────→ imu_link    │
         ├───────→ laser_link  │
         ├───────→ wheel_left  │
         └───────→ wheel_right │
                               │
                    [robot_localization/ekf_node]
```

## Paramètres Clés

### Fréquence
- **EKF**: 50 Hz
- **IMU**: 100 Hz
- **Wheel Odom**: ~50 Hz (dépend du contrôleur)

### Covariance
- **Process Noise**: Incertitude du modèle de prédiction
- **Initial Estimate**: Incertitude initiale de l'état
- Plus les valeurs sont élevées → Plus le filtre est "souple"
- Plus les valeurs sont basses → Plus le filtre est "strict"

## Topics ROS2 Importants

| Topic              | Type                  | Description                    |
|--------------------|-----------------------|--------------------------------|
| /imu/data          | sensor_msgs/Imu       | Données brutes de l'IMU        |
| /diff_cont/odom    | nav_msgs/Odometry     | Odométrie des roues            |
| /odometry/local    | nav_msgs/Odometry     | Odométrie fusionnée (EKF)      |
| /scan              | sensor_msgs/LaserScan | Données du Lidar               |

## Commandes Utiles

```bash
# Visualiser le graphe TF
ros2 run tf2_tools view_frames
evince frames.pdf

# Inspecter la transformation odom → base_link
ros2 run tf2_ros tf2_echo odom base_link

# Vérifier la fréquence des topics
ros2 topic hz /imu/data
ros2 topic hz /odometry/local
ros2 topic hz /diff_cont/odom

# Voir les paramètres de l'EKF
ros2 param list /ekf_filter_node
ros2 param get /ekf_filter_node odom0
ros2 param get /ekf_filter_node imu0

# Enregistrer des données pour analyse
ros2 bag record /imu/data /diff_cont/odom /odometry/local

# Comparer les odométries
ros2 topic echo /diff_cont/odom --field pose.pose.position
ros2 topic echo /odometry/local --field pose.pose.position
```

## Métriques de Performance

### Amélioration Attendue

| Métrique                    | Sans IMU  | Avec IMU  | Amélioration |
|-----------------------------|-----------|-----------|--------------|
| Précision orientation       | ±5°       | ±1°       | 80%          |
| Dérive sur 10m              | 0.5m      | 0.1m      | 80%          |
| Détection glissement        | ❌        | ✅        | -            |
| Latence odométrie           | 50ms      | 20ms      | 60%          |
| Fréquence mise à jour       | 50Hz      | 50Hz      | -            |

## Troubleshooting

### Problème: L'EKF ne publie pas
```bash
# Vérifier que le node tourne
ros2 node list | grep ekf

# Vérifier les logs
ros2 node info /ekf_filter_node

# Vérifier les sources de données
ros2 topic hz /imu/data
ros2 topic hz /diff_cont/odom
```

### Problème: Mauvaise fusion
```bash
# Ajuster les covariances dans ekf_params.yaml
# Augmenter process_noise_covariance pour plus de souplesse
# Vérifier que imu0_remove_gravitational_acceleration: true
```

### Problème: TF non disponible
```bash
# Vérifier que publish_tf: true dans ekf_params.yaml
# Vérifier les frames
ros2 run tf2_ros tf2_echo odom base_link
```
