# 🦾 MoveIt2 - Quick Reference Card

## 🚀 Installation (One-Time)
```bash
bash install_moveit.sh
```

## 🎮 Launch Options

### Full System (Recommended for First Time)
```bash
ros2 launch my_robot_controller moveit.launch.py
```
Launches: Gazebo + Controllers + MoveIt + RViz

### Add to Existing Simulation
```bash
# Terminal 1: Already running SLAM/Navigation
ros2 launch my_robot_controller slam_mapping.launch.py

# Terminal 2: Add MoveIt
ros2 launch my_robot_controller moveit_only.launch.py
```

## 🎯 Control Commands

### Arm Positions
```bash
ros2 run my_robot_controller arm_moveit_control.py home       # 🏠 Rest
ros2 run my_robot_controller arm_moveit_control.py extended   # ➡️  Forward
ros2 run my_robot_controller arm_moveit_control.py tucked     # 📦 Compact
ros2 run my_robot_controller arm_moveit_control.py ready      # 🎯 Pick ready
```

### Gripper
```bash
ros2 run my_robot_controller arm_moveit_control.py open       # 🤚 Open
ros2 run my_robot_controller arm_moveit_control.py close      # ✊ Close
ros2 run my_robot_controller arm_moveit_control.py half_open  # 🖐️  Half
```

### Demos
```bash
ros2 run my_robot_controller arm_moveit_control.py demo       # 🎬 Pick & place
ros2 run my_robot_controller interactive_arm_control.py       # 🎮 Interactive menu
```

## 📋 Predefined Poses

| Name | Shoulder | Elbow | Wrist | Use Case |
|------|----------|-------|-------|----------|
| home | 0° | 0° | 0° | Rest position |
| extended | 0° | 90° | 0° | Reach forward |
| tucked | -69° | -115° | 0° | Storage |
| ready | 29° | 46° | 0° | Pick objects |

## 🔍 Diagnostics

```bash
# Check controllers
ros2 control list_controllers

# Check move_group
ros2 node list | grep move_group

# Check joint states  
ros2 topic echo /joint_states
```

## 🐛 Quick Fixes

**Planning failed?**
```bash
# Check if unreachable - try different pose
ros2 run my_robot_controller arm_moveit_control.py home
```

**Controllers missing?**
```bash
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller
```

**Python import error?**
```bash
pip3 install moveit
```

## 📖 Documentation

- **Quick Start**: [MOVEIT_QUICK_START.md](docs/MOVEIT_QUICK_START.md)
- **Full Guide**: [MOVEIT_GUIDE.md](docs/MOVEIT_GUIDE.md)
- **Install Help**: [MOVEIT_INSTALLATION.md](docs/MOVEIT_INSTALLATION.md)
- **Summary**: [MOVEIT_SUMMARY.md](MOVEIT_SUMMARY.md)

## ⚡ Typical Workflow

```bash
# 1. Launch system
ros2 launch my_robot_controller moveit.launch.py

# 2. New terminal - Test
source ~/ROS2_Project/install/setup.bash
ros2 run my_robot_controller arm_moveit_control.py home

# 3. Run demo
ros2 run my_robot_controller arm_moveit_control.py demo

# 4. Interactive control
ros2 run my_robot_controller interactive_arm_control.py
```

---

**🎉 Your arm is ready for intelligent motion planning!**
