# 🚀 Quick Start - IMU Integration

## Installation Rapide

```bash
# 1. Installer robot_localization
sudo apt install ros-humble-robot-localization

# 2. Build
cd ~/ROS_PROJECT
colcon build --packages-select my_robot_controller
source install/setup.bash

# 3. Test
./test_imu_integration.sh
```

## Lancement

```bash
# Simulation avec IMU (automatique)
ros2 launch my_robot_controller launch_sim.launch.py

# SLAM avec IMU
ros2 launch my_robot_controller slam_mapping.launch.py

# Navigation avec IMU
ros2 launch my_robot_controller launch_navigation.launch.py
```

## Vérification Rapide

```bash
# Vérifier que l'EKF fonctionne
ros2 node list | grep ekf

# Vérifier l'odométrie fusionnée
ros2 topic hz /odometry/local
# Devrait afficher: ~50 Hz

# Vérifier l'IMU
ros2 topic hz /imu/data
# Devrait afficher: ~100 Hz
```

## Topics Importants

| Topic              | Description                    |
|--------------------|--------------------------------|
| `/imu/data`        | Données brutes IMU             |
| `/diff_cont/odom`  | Odométrie des roues            |
| `/odometry/local`  | Odométrie fusionnée (EKF) ⭐   |

## Fichiers de Configuration

| Fichier                          | Description                    |
|----------------------------------|--------------------------------|
| `config/ekf_params.yaml`         | Configuration EKF              |
| `config/nav2_params.yaml`        | Navigation (utilise EKF)       |
| `description/imu.xacro`          | Description matérielle IMU     |

## Ajustements Courants

### Robot oscille
```yaml
# config/ekf_params.yaml
frequency: 30.0  # Réduire de 50 → 30
```

### Orientation dérive
```yaml
# config/ekf_params.yaml
imu0_config: [..., true, true, true, ...]  # S'assurer que orientation est activée
```

### Réponse lente
```yaml
# config/ekf_params.yaml
frequency: 100.0  # Augmenter de 50 → 100
```

## Documentation Complète

- 📖 [Guide d'intégration](docs/IMU_INTEGRATION_GUIDE.md)
- 🏗️ [Architecture](docs/IMU_ARCHITECTURE.md)
- ⚙️ [Ajustement paramètres](docs/EKF_TUNING_GUIDE.md)
- 📝 [Résumé](IMU_INTEGRATION_SUMMARY.md)

## Aide

```bash
# Script de test complet
./test_imu_integration.sh

# Voir les logs de l'EKF
ros2 node info /ekf_filter_node

# Debug TF
ros2 run tf2_tools view_frames
```

---

**✅ L'IMU est intégré et prêt à être utilisé !**
