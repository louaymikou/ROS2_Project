# Blue Line Follower Robot

A ROS2 package for a 4-wheeled robot that follows blue lines using computer vision in Gazebo simulation.

## Features

- **4-Wheeled Robot**: Differential drive robot with realistic physics
- **Camera Sensor**: Front-facing camera for line detection
- **OpenCV Integration**: HSV color space filtering and contour detection
- **Proportional Control**: Simple P-controller for smooth line following
- **Gazebo Simulation**: Complete simulation environment with blue line track

## Algorithm

The line following algorithm uses:
1. **HSV Color Space Conversion**: Converts camera image from BGR to HSV for robust color detection
2. **Binary Masking**: Creates a mask isolating blue pixels (HSV range: 100-130 hue)
3. **Contour Detection**: Uses OpenCV's `findContours` to detect the blue line
4. **Centroid Calculation**: Computes the center point of the detected line using moments
5. **Proportional Control**: Adjusts angular velocity based on error from image center

## Installation

```bash
cd ~/Downloads/line_follower_test/line_follower_ws
colcon build --packages-select blue_line_follower
source install/setup.bash
```

## Usage

### Lancer la simulation

```bash
ros2 launch blue_line_follower simulation.launch.py
```

Ceci démarre:
- Gazebo avec la piste de ligne bleue
- Le robot 4 roues à la position de départ
- Le nœud de suivi de ligne
- **ATTENTION**: Le robot ne bouge PAS automatiquement, vous devez activer le mouvement

### Commandes de contrôle

#### Activer/Désactiver le mouvement

**Démarrer le robot** (activer le suivi de ligne):
```bash
ros2 service call /enable_movement std_srvs/srv/SetBool "{data: true}"
```

**Arrêter le robot** (désactiver le suivi de ligne):
```bash
ros2 service call /enable_movement std_srvs/srv/SetBool "{data: false}"
```

#### Changer de direction

**Avancer** (utiliser la caméra avant):
```bash
ros2 service call /set_forward_direction std_srvs/srv/SetBool "{data: true}"
```

**Reculer** (utiliser la caméra arrière):
```bash
ros2 service call /set_forward_direction std_srvs/srv/SetBool "{data: false}"
```

### Séquence complète typique

1. Lancer la simulation:
   ```bash
   ros2 launch blue_line_follower simulation.launch.py
   ```

2. Activer le mouvement en avant:
   ```bash
   ros2 service call /enable_movement std_srvs/srv/SetBool "{data: true}"
   ```

3. Pour inverser la direction:
   ```bash
   ros2 service call /set_forward_direction std_srvs/srv/SetBool "{data: false}"
   ```

4. Pour arrêter:
   ```bash
   ros2 service call /enable_movement std_srvs/srv/SetBool "{data: false}"
   ```

## Package Structure

```
blue_line_follower/
├── blue_line_follower/
│   ├── __init__.py
│   └── line_follower_node.py    # Main line following node
├── launch/
│   └── simulation.launch.py      # Launch file for simulation
├── urdf/
│   └── line_follower_robot.urdf.xacro  # Robot description
├── worlds/
│   └── blue_line_track.world     # Gazebo world with blue line
├── package.xml
├── setup.py
└── README.md
```

## Parameters

You can adjust these parameters in `line_follower_node.py`:

- `LINEAR_SPEED`: Forward velocity (default: 0.2 m/s)
- `KP`: Proportional gain for turning (default: 0.015)
- `MIN_AREA_TRACK`: Minimum contour area to consider (default: 50 pixels)
- `lower_blue`: HSV lower bound [100, 50, 50]
- `upper_blue`: HSV upper bound [130, 255, 255]

## Topics

- `/camera/image_raw` (sensor_msgs/Image): Camera feed
- `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/odom` (nav_msgs/Odometry): Odometry data

## Requirements

- ROS2 Humble
- Gazebo
- OpenCV (via cv_bridge)
- Python 3

## Based On

This implementation is based on the line following algorithm from The Construct's BOTBOX training materials, using OpenCV and ROS2 integration for autonomous navigation.
