# 🤖 Guide du Système Hybride - Alternative au LIDAR

## 📋 Vue d'Ensemble

Ce guide explique comment utiliser le système de capteurs hybride qui remplace le LIDAR par:
- **📷 Caméra Raspberry Pi V2** - SLAM visuel
- **📡 8× Capteurs Ultrasoniques HC-SR04** - Détection d'obstacles 360°
- **🧭 IMU MPU6050** - Orientation et stabilisation

### Coût Total: ~105€ (vs ~200-400€ pour un LIDAR)

---

## 🚀 Lancement Rapide

### Simulation Complète (Gazebo + RViz + Nav2 + SLAM)
```bash
# Dans Docker ou environnement ROS2
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash

# Lancer la simulation hybride complète
ros2 launch my_robot_controller hybrid_simulation.launch.py
```

### Options de Lancement
```bash
# Mode headless (sans GUI Gazebo) - plus rapide
ros2 launch my_robot_controller hybrid_simulation.launch.py gui:=false

# Sans SLAM (navigation avec carte existante)
ros2 launch my_robot_controller hybrid_simulation.launch.py use_slam:=false

# Sans Nav2 (juste simulation + visualisation)
ros2 launch my_robot_controller hybrid_simulation.launch.py use_nav:=false

# Sans RViz
ros2 launch my_robot_controller hybrid_simulation.launch.py use_rviz:=false
```

---

## 📁 Fichiers Créés

### Description Robot (URDF/Xacro)
| Fichier | Description |
|---------|-------------|
| [description/robot_hybrid.urdf.xacro](src/my_robot_controller/description/robot_hybrid.urdf.xacro) | Robot complet avec capteurs hybrides |
| [description/hybrid_sensors.xacro](src/my_robot_controller/description/hybrid_sensors.xacro) | Définition des capteurs (caméra, IMU, 8 ultrasons) |

### Configuration
| Fichier | Description |
|---------|-------------|
| [config/ekf_config.yaml](src/my_robot_controller/config/ekf_config.yaml) | Fusion de capteurs EKF (odométrie + IMU) |
| [config/nav2_hybrid_params.yaml](src/my_robot_controller/config/nav2_hybrid_params.yaml) | Navigation adaptée aux ultrasons |
| [config/slam_hybrid_params.yaml](src/my_robot_controller/config/slam_hybrid_params.yaml) | SLAM Toolbox pour données ultrasoniques |

### Scripts
| Fichier | Description |
|---------|-------------|
| [scripts/ultrasonic_aggregator.py](src/my_robot_controller/scripts/ultrasonic_aggregator.py) | Combine 8 Range → 1 LaserScan |

### Launch & Visualisation
| Fichier | Description |
|---------|-------------|
| [launch/hybrid_simulation.launch.py](src/my_robot_controller/launch/hybrid_simulation.launch.py) | Launch principal simulation hybride |
| [rviz/hybrid_robot.rviz](src/my_robot_controller/rviz/hybrid_robot.rviz) | Configuration RViz pour système hybride |

---

## 🔧 Architecture du Système

```
┌─────────────────────────────────────────────────────────────────────┐
│                         SYSTÈME HYBRIDE                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │
│  │  Caméra     │    │    IMU      │    │ 8× Ultrasons│             │
│  │  RPi V2     │    │  MPU6050    │    │  HC-SR04    │             │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘             │
│         │                  │                  │                     │
│         ▼                  ▼                  ▼                     │
│  /camera/image_raw   /imu/data        /ultrasonic/*                │
│         │                  │                  │                     │
│         │           ┌──────┴──────┐   ┌──────┴──────┐              │
│         │           │     EKF     │   │ Aggregator  │              │
│         │           │   Fusion    │   │  8 → 1      │              │
│         │           └──────┬──────┘   └──────┬──────┘              │
│         │                  │                  │                     │
│         │                  ▼                  ▼                     │
│         │         /odometry/filtered   /ultrasonic_scan            │
│         │                  │                  │                     │
│         │                  └────────┬─────────┘                    │
│         │                           │                               │
│         │                           ▼                               │
│         │                    ┌─────────────┐                        │
│         │                    │  SLAM       │                        │
│         │                    │  Toolbox    │                        │
│         │                    └──────┬──────┘                        │
│         │                           │                               │
│         │                           ▼                               │
│         │                    ┌─────────────┐                        │
│         │                    │    Nav2     │                        │
│         │                    │  Navigation │                        │
│         └────────────────────┤             │                        │
│                              └─────────────┘                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 📊 Topics ROS2

### Capteurs
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Image RGB 640×480 @ 30Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration caméra |
| `/imu/data` | `sensor_msgs/Imu` | Accéléromètre + Gyroscope @ 100Hz |
| `/ultrasonic/ultrasonic_front` | `sensor_msgs/Range` | Capteur avant |
| `/ultrasonic/ultrasonic_front_left` | `sensor_msgs/Range` | Capteur avant-gauche |
| `/ultrasonic/ultrasonic_left` | `sensor_msgs/Range` | Capteur gauche |
| `/ultrasonic/ultrasonic_rear_left` | `sensor_msgs/Range` | Capteur arrière-gauche |
| `/ultrasonic/ultrasonic_rear` | `sensor_msgs/Range` | Capteur arrière |
| `/ultrasonic/ultrasonic_rear_right` | `sensor_msgs/Range` | Capteur arrière-droit |
| `/ultrasonic/ultrasonic_right` | `sensor_msgs/Range` | Capteur droit |
| `/ultrasonic/ultrasonic_front_right` | `sensor_msgs/Range` | Capteur avant-droit |

### Données Traitées
| Topic | Type | Description |
|-------|------|-------------|
| `/ultrasonic_scan` | `sensor_msgs/LaserScan` | Scan agrégé (8 capteurs → 360°) |
| `/odometry/filtered` | `nav_msgs/Odometry` | Odométrie fusionnée (EKF) |

### Navigation
| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Carte SLAM |
| `/plan` | `nav_msgs/Path` | Chemin global |
| `/local_plan` | `nav_msgs/Path` | Chemin local |
| `/cmd_vel` | `geometry_msgs/Twist` | Commandes vélocité |

---

## 🎯 Tests de Validation

### 1. Vérifier les Capteurs
```bash
# Dans un nouveau terminal

# Vérifier la caméra
ros2 topic echo /camera/image_raw --once

# Vérifier l'IMU
ros2 topic echo /imu/data --once

# Vérifier un ultrason
ros2 topic echo /ultrasonic/ultrasonic_front --once

# Vérifier le scan agrégé
ros2 topic echo /ultrasonic_scan --once

# Vérifier l'odométrie fusionnée
ros2 topic echo /odometry/filtered --once
```

### 2. Vérifier la Fréquence
```bash
# Fréquence du scan ultrasonique (devrait être ~20 Hz)
ros2 topic hz /ultrasonic_scan

# Fréquence de l'IMU (devrait être ~100 Hz)
ros2 topic hz /imu/data

# Fréquence de l'odométrie fusionnée (devrait être ~50 Hz)
ros2 topic hz /odometry/filtered
```

### 3. Tester la Navigation
```bash
# Envoyer un goal de navigation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 1.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

### 4. Contrôle Manuel (Téléopération)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped
```

---

## 📈 Performances Attendues

| Métrique | LIDAR | Système Hybride | Notes |
|----------|-------|-----------------|-------|
| **Précision SLAM** | 95% | 80-85% | Acceptable pour la plupart des cas |
| **Détection obstacles** | 360° continu | 360° discret (45°) | 8 points vs 720 points |
| **Portée** | 15m | 4m | Suffisant pour intérieur |
| **Fréquence** | 20Hz | 20Hz | Équivalent |
| **Coût** | ~200-400€ | ~105€ | **50-70% moins cher** |

---

## ⚠️ Limitations et Solutions

### Limitations des Ultrasons
| Problème | Solution |
|----------|----------|
| Angle de détection étroit (15°) | 8 capteurs à 45° pour couverture 360° |
| Réflexions sur surfaces lisses | Vitesse réduite + marge de sécurité |
| Portée limitée (4m) | Navigation intérieure uniquement |

### Limitations de la Caméra
| Problème | Solution |
|----------|----------|
| Sensible à la lumière | Éclairage constant recommandé |
| Pas de profondeur native | Combinaison avec ultrasons |
| SLAM visuel moins précis | EKF fusion avec IMU |

---

## 🔄 Comparaison: LIDAR vs Hybride

### Simulation LIDAR (existante)
```bash
ros2 launch my_robot_controller autonomous_mission.launch.py
```

### Simulation Hybride (nouvelle)
```bash
ros2 launch my_robot_controller hybrid_simulation.launch.py
```

### Différences Clés
1. **URDF**: `robot.urdf.xacro` → `robot_hybrid.urdf.xacro`
2. **Scan topic**: `/scan` → `/ultrasonic_scan`
3. **Odométrie**: `/diff_cont/odom` → `/odometry/filtered` (avec IMU)
4. **Costmap source**: LaserScan direct → LaserScan agrégé

---

## 🛠️ Déploiement Hardware

Quand vous êtes satisfait de la simulation, suivez ces étapes pour le hardware réel:

1. **Acheter le matériel** (~105€):
   - Raspberry Pi Camera V2 (~25€)
   - 8× HC-SR04 (~2€ chacun = 16€)
   - MPU6050 (~5€)
   - Raspberry Pi 4 (~60€) si pas déjà possédé

2. **Modifier les drivers**:
   - Remplacer plugins Gazebo par drivers hardware
   - Caméra: `v4l2_camera` ou `raspicam2_node`
   - IMU: `mpu6050_driver`
   - Ultrasons: Driver GPIO personnalisé

3. **Calibration**:
   - Calibrer la caméra (checkerboard)
   - Calibrer l'IMU (placement horizontal)
   - Ajuster les offsets ultrasoniques

---

## 📝 Troubleshooting

### Problème: Pas de données de capteurs
```bash
# Vérifier les topics disponibles
ros2 topic list | grep -E "camera|imu|ultrasonic"

# Si vide, vérifier le robot description
ros2 param get /robot_state_publisher robot_description
```

### Problème: SLAM ne fonctionne pas
```bash
# Vérifier le scan
ros2 topic echo /ultrasonic_scan --once

# Vérifier les TF
ros2 run tf2_ros tf2_echo map base_link
```

### Problème: Navigation échoue
```bash
# Vérifier le costmap
ros2 topic echo /local_costmap/costmap --once

# Vérifier l'odométrie
ros2 topic echo /odometry/filtered --once
```

---

## 📚 Ressources

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/)
- [HC-SR04 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

---

**Créé pour le projet ROS2 Mobile Manipulator - Décembre 2024**
