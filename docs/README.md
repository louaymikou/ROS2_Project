# ROS2 Mobile Robot Project

## 🤖 Project Overview

Mobile robot with teleoperation and mapping capabilities:

- 4-wheel differential drive base
- 3-DOF robotic arm + 2-finger gripper (hardware only)
- SLAM-based mapping (LIDAR sensor)
- Nav2 navigation support
- Teleoperation control (keyboard/PS4)

---

## 📋 Prerequisites

### Install Dependencies

```bash
# ROS2 Humble + Gazebo
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs

# Navigation & SLAM
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup

# ros2_control
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

# Teleop tools
sudo apt install -y ros-humble-teleop-twist-keyboard

# PS4 controller support (optional)
sudo apt install -y ros-humble-joy
```

### Build Workspace

```bash
cd ~/ROS_PROJECT
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage Modes

### 1️⃣ **BASIC SIMULATION**

Launch Gazebo simulation with the robot:

```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim.launch.py
```

---

### 2️⃣ **KEYBOARD CONTROL**

Launch simulation with keyboard controller:

```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py
```

**Controls:**
- `w` - Forward
- `s` - Backward
- `a` - Turn left
- `d` - Turn right
- `x` - Stop
- `q` - Quit

---

### 3️⃣ **PS4 CONTROLLER**

Launch simulation with PS4 controller:

```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim_with_ps4.launch.py
```

**Controls:**
- Left Stick - Drive (forward/backward/turn)
- Right Stick - Arm control (up/down)
- R1/L1 - Gripper control

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
python3 src/my_robot_controller/nodes/controllers/keyboard_controller.py
```

Or use teleop_twist_keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

---

### 5️⃣ **SLAM MAPPING**

Create a map of the environment:

**Terminal 1 - SLAM Launch:**

```bash
source install/setup.bash
ros2 launch my_robot_controller slam_mapping.launch.py
```

**Terminal 2 - Drive Around:**

```bash
python3 src/my_robot_controller/nodes/controllers/keyboard_controller.py
# OR use teleop_twist_keyboard
```

**Terminal 3 - Save Map:**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ROS_PROJECT/maps/my_map
```

---

## 📊 System Architecture

```
Teleoperation (Keyboard/PS4)
    ↓
ros2_control (diff_cont, arm_controller, gripper_controller)
    ↓
Gazebo Simulation + LIDAR Sensor
    ↓
SLAM (slam_toolbox) → Map Generation
    ↓
Nav2 (optional navigation)
```

---

## 🛠️ Troubleshooting

### "Package not found" error

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Robot doesn't move

```bash
# Verify controllers
ros2 control list_controllers

# Should show: diff_cont, arm_controller, gripper_controller, joint_broad
```

### Controllers not loading

```bash
# Check controller manager
ros2 control list_hardware_interfaces
```

### LIDAR not working

Check that the LIDAR sensor is publishing data:

```bash
ros2 topic echo /scan
```

---

## 📁 Project Structure

```
ROS_PROJECT/
├── src/my_robot_controller/
│   ├── nodes/
│   │   ├── controllers/       # Keyboard & PS4 controllers
│   │   ├── mappers/          # SLAM mapping utilities
│   │   └── navigation/       # Navigation helpers
│   ├── launch/               # Launch files
│   ├── config/               # Configuration files
│   ├── description/          # URDF robot description
│   ├── worlds/              # Gazebo world files
│   └── models/              # 3D models
├── maps/                     # Generated maps
├── docs/                     # Documentation
└── config/                  # Global config files
```

---

## 🎯 Available Launch Files

| Launch File | Description |
|------------|-------------|
| `launch_sim.launch.py` | Basic simulation only |
| `launch_sim_with_keyboard.launch.py` | Simulation + keyboard control |
| `launch_sim_with_ps4.launch.py` | Simulation + PS4 controller |
| `slam_mapping.launch.py` | SLAM mapping mode |
| `launch_mapping.launch.py` | Alternative mapping launch |

---

## 🎮 ros2_control Controllers

The robot uses the following controllers:

1. **diff_cont** - Differential drive controller for base mobility
2. **joint_broad** - Joint state broadcaster
3. **arm_controller** - Position controller for robotic arm joints
4. **gripper_controller** - Position controller for gripper fingers

All controllers are automatically spawned during launch.

---

## 📖 Quick Start (First Time Setup)

```bash
# 1. Install dependencies
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control

# 2. Build workspace
cd ~/ROS_PROJECT
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Run simulation with keyboard control
ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py
```

---

**Project Status:** ✅ Complete - Teleoperation and SLAM mapping operational  
**Last Updated:** January 2, 2026  
**Branch:** `ikram`
