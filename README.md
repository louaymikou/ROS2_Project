# ROS2 Mobile Manipulator Project

## 🤖 Project Overview

Autonomous pick-and-place mobile manipulator with:

- 4-wheel differential drive base
- 3-DOF robotic arm + 2-finger gripper
- SLAM-based navigation (LIDAR sensor)
- Nav2 autonomous navigation
- Action-based control architecture

---

## 🐳 Docker (Recommended)

Run the project with Docker - no ROS2 installation required!

> 📺 **RViz2 Visualization:** RViz2 se lance automatiquement pour visualiser le robot, le LIDAR, la carte SLAM et les trajectoires. Voir [RVIZ_GUIDE.md](RVIZ_GUIDE.md) pour plus de détails.

### Prerequisites

**Docker Engine (Recommended for GUI/Joystick support):**

```bash
# Install Docker Engine (NOT Docker Desktop)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Log out and back in for group changes
```

> ⚠️ **Docker Desktop vs Docker Engine**: Docker Desktop on Linux has limitations with X11 forwarding and device passthrough. Use Docker Engine for full GUI (Gazebo/RViz2) and PS4 controller support.

### X11 Setup (Required for GUI)

**Linux:**

```bash
xhost +local:docker
```

**WSL2 (Windows):**

1. Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [X410](https://x410.dev/)
2. Launch with "Disable access control" checked
3. Set DISPLAY:

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
```

### Quick Start

```bash
# Build image (first time only)
docker compose build

# Allow X11 access
xhost +local:docker

# Run autonomous mission (default)
docker compose up ros2
```

### Running Different Modes

| Mode | Command |
|------|---------|
| **Autonomous Mission** | `docker compose up ros2` |
| **Keyboard Control** | `docker compose up keyboard` |
| **PS4 Controller** | `docker compose up ps4` |
| **SLAM Mapping** | `docker compose up slam` |
| **Basic Simulation** | `docker compose up sim` |
| **Shell (Dev)** | `docker compose run --rm shell` |

### Running Mission Orchestrator

```bash
# Terminal 1: Start simulation (RViz2 included by default)
docker compose up ros2

# To disable RViz2 (save resources):
# docker compose run --rm ros2 ros2 launch my_robot_controller autonomous_mission.launch.py rviz:=false

# Terminal 2: Wait 15-20s, then run mission
docker compose exec ros2 ros2 run my_robot_controller mission_orchestrator.py
```

**RViz2 Visualization:** Vous verrez le robot en 3D, le LIDAR, la carte SLAM et les trajectoires en temps réel!

### Troubleshooting Docker

| Issue | Solution |
|-------|----------|
| No GUI display | Run `xhost +local:docker` |
| Joystick not detected | Ensure `/dev/input` exists and controller is connected |
| Permission denied | Add user to docker group: `sudo usermod -aG docker $USER` |
| Build fails | Check internet connection, retry with `docker compose build --no-cache` |

---

## 📋 Prerequisites (Native Installation)

### Install Dependencies

```bash
# ROS2 Humble + Gazebo
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs

# Navigation & SLAM
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup

# ros2_control
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

# Teleop tools (optional)
sudo apt install -y ros-humble-teleop-twist-keyboard
```

### Build Workspace

```bash
cd ~/ROS2_Project/ROS2_Project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage Modes

### 1️⃣ **AUTONOMOUS MISSION** (Recommended - New!)

**Pick object from Point A → Place at Point B → Return to A**

**Terminal 1 - Launch System:**

```bash
cd ~/ROS2_Project/ROS2_Project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_controller autonomous_mission.launch.py
```

**Wait ~15-20 seconds for Nav2 initialization**

**Terminal 2 - Start Mission:**

```bash
cd ~/ROS2_Project/ROS2_Project
source install/setup.bash
ros2 run my_robot_controller mission_orchestrator.py
```

**Watch the robot autonomously:**

- Navigate to cube location (Point A: 0, 5.5)
- Pick cube with arm + gripper
- Navigate to drop-off (Point B: 3, 0)
- Place cube
- Return to Point A

**Mission duration:** ~2-3 minutes

---

### 2️⃣ **MANUAL CONTROL - PS4 Controller**

```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim_with_ps4.launch.py
```

---

### 3️⃣ **MANUAL CONTROL - Keyboard (All-in-One)**

```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py
```

---

### 4️⃣ **MANUAL CONTROL - Separate Terminals**

**Terminal 1 - Simulation:**

```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim.launch.py
```

**Terminal 2 - Keyboard Control:**

```bash
source install/setup.bash
python3 src/my_robot_controller/keyboard_controller.py
```

---

### 5️⃣ **MANUAL CONTROL - Teleop + Arm**

**Terminal 1 - Simulation:**

```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim.launch.py
```

**Terminal 2 - Base Teleop:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

**Terminal 3 - Arm Control:**

```bash
python3 src/my_robot_controller/simple_arm_control.py
```

---

### 6️⃣ **SLAM MAPPING** (Build Map First Time)

**Terminal 1 - SLAM Launch:**

```bash
source install/setup.bash
ros2 launch my_robot_controller slam_mapping.launch.py
```

**Terminal 2 - Drive Around:**

```bash
python3 src/my_robot_controller/keyboard_controller.py
# OR use teleop_twist_keyboard
```

**Terminal 3 - Save Map:**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

---

## 📊 System Architecture

```
Mission Orchestrator (A→B→A logic)
    ↓
Pick/Place Action Server (navigation + manipulation)
    ↓
Nav2 (path planning) + Arm Action Server
    ↓
SLAM (localization) + ros2_control (arm/gripper/base)
    ↓
Gazebo Simulation + LIDAR Sensor
```

---

## 🛠️ Troubleshooting

### "Package not found" error

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Nav2 won't start

- Wait longer (up to 30 seconds on slower machines)
- Check SLAM is publishing `/map` topic: `ros2 topic list | grep map`

### Robot doesn't move

```bash
# Verify controllers
ros2 control list_controllers

# Should show: diff_cont, arm_controller, gripper_controller, joint_broad
```

### High resource usage

- See `RESOURCE_OPTIMIZATION.md` for 6GB RAM VM tuning
- Gazebo runs headless by default (no GUI to save memory)

---

## 📚 Documentation

- `AUTONOMOUS_SYSTEM_PLAN.md` - Complete technical architecture
- `SETUP_AND_USAGE_GUIDE.md` - Detailed setup instructions
- `RESOURCE_OPTIMIZATION.md` - Performance tuning for low-spec VMs
- `QUICK_REFERENCE.md` - Command cheat sheet
- `QUICK_SUMMARY.md` - High-level overview
- `RVIZ_GUIDE.md` - Guide d'utilisation de RViz2 pour la visualisation
- `DOCKER_GUIDE.md` - Docker deployment guide

---

## 🎯 Quick Start (First Time Setup)

```bash
# 1. Install dependencies
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control

# 2. Build workspace
cd ~/ROS2_Project/ROS2_Project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Run autonomous mission
ros2 launch my_robot_controller autonomous_mission.launch.py
# Wait 15s, then in new terminal:
ros2 run my_robot_controller mission_orchestrator.py
```

---

**Project Status:** ✅ Complete - Autonomous pick-and-place system operational  
**Last Updated:** December 20, 2025  
**Branch:** `lamiae` (development), `main` (stable)
