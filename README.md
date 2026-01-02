# ROS2 Mobile Manipulator Project

🤖 Autonomous pick-and-place mobile manipulator with SLAM navigation and robotic arm control.

## 📁 Project Structure

```
ROS_PROJECT/
├── docs/                    # Documentation
│   ├── README.md           # Main project documentation
│   └── INSTALLATION_GUIDE.md
├── maps/                    # Generated maps
│   ├── my_robot_map.pgm
│   ├── my_robot_map.png
│   └── my_robot_map.yaml
├── config/                  # Configuration files
│   ├── slam_params.yaml
│   └── nav2_params/
├── src/                     # Source code
│   └── my_robot_controller/
└── .gitignore

```

## 🚀 Quick Start

See [docs/README.md](docs/README.md) for complete documentation.

### Installation

```bash
# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Run autonomous mission
ros2 launch my_robot_controller autonomous_mission.launch.py
```

## 📖 Documentation

- **[Main Documentation](docs/README.md)** - Complete usage guide
- **[Installation Guide](docs/INSTALLATION_GUIDE.md)** - Setup instructions

## 🎯 Features

- ✅ Autonomous navigation with Nav2
- ✅ SLAM mapping with slam_toolbox
- ✅ Pick and place operations
- ✅ PS4 controller support
- ✅ Keyboard control

---

**Last Updated:** January 2, 2026  
**Branch:** `ikram`
