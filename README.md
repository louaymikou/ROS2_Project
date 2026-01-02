# ROS2 Mobile Robot Project

🤖 Mobile robot with teleoperation, SLAM mapping, and navigation capabilities.

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

# Run simulation with keyboard control
ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py
```

## 📖 Documentation

- **[Main Documentation](docs/README.md)** - Complete usage guide
- **[Installation Guide](docs/INSTALLATION_GUIDE.md)** - Setup instructions

## 🎯 Features

- ✅ SLAM mapping with slam_toolbox
- ✅ Teleoperation (keyboard/PS4 controller)
- ✅ Nav2 navigation support
- ✅ ros2_control for robot control
- ✅ Gazebo simulation

---

**Last Updated:** January 2, 2026  
**Branch:** `ikram`
