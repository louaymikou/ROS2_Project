# 🦾 Quick MoveIt Control Reference

Fast reference for MoveIt arm control commands.

---

## 🚀 Launch MoveIt

### Full Launch (Gazebo + MoveIt + RViz)
```bash
ros2 launch my_robot_controller moveit.launch.py
```

### Add to Existing Simulation
```bash
# Terminal 1: Start SLAM or Navigation first
ros2 launch my_robot_controller slam_mapping.launch.py

# Terminal 2: Add MoveIt
ros2 launch my_robot_controller moveit_only.launch.py
```

---

## 🎮 Quick Commands

### Predefined Arm Poses
```bash
ros2 run my_robot_controller arm_moveit_control.py home        # Home position
ros2 run my_robot_controller arm_moveit_control.py extended    # Extended forward
ros2 run my_robot_controller arm_moveit_control.py tucked      # Compact pose
ros2 run my_robot_controller arm_moveit_control.py ready       # Ready to pick
```

### Gripper Control
```bash
ros2 run my_robot_controller arm_moveit_control.py open        # Open gripper
ros2 run my_robot_controller arm_moveit_control.py close       # Close gripper
ros2 run my_robot_controller arm_moveit_control.py half_open   # Half open
```

### Demonstrations
```bash
ros2 run my_robot_controller arm_moveit_control.py demo        # Pick & place demo
ros2 run my_robot_controller interactive_arm_control.py        # Interactive menu
```

---

## 📋 Predefined Poses

| Command | Shoulder | Elbow | Wrist | Description |
|---------|----------|-------|-------|-------------|
| `home` | 0° | 0° | 0° | Rest position |
| `extended` | 0° | 90° | 0° | Arm forward |
| `tucked` | -69° | -115° | 0° | Compact |
| `ready` | 29° | 46° | 0° | Ready to pick |

---

## 🔧 Installation (One-Time)

```bash
# Install MoveIt2
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-planners-ompl

# Build workspace
cd ~/ROS2_Project
colcon build
source install/setup.bash
```

---

## 🎯 Common Workflows

### Pick and Place
```bash
# Terminal 1: Start simulation
ros2 launch my_robot_controller moveit.launch.py

# Terminal 2: Run demo
ros2 run my_robot_controller arm_moveit_control.py demo
```

### Custom Control
```bash
# Terminal 1: Start simulation
ros2 launch my_robot_controller navigation.launch.py

# Terminal 2: MoveIt
ros2 launch my_robot_controller moveit_only.launch.py

# Terminal 3: Interactive control
ros2 run my_robot_controller interactive_arm_control.py
```

---

## 🐛 Quick Troubleshooting

### Controllers not working?
```bash
ros2 control list_controllers
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller
```

### Planning failed?
```bash
# Check joint states
ros2 topic echo /joint_states

# Verify move_group is running
ros2 node list | grep move_group
```

---

**📖 Full Guide:** See [MOVEIT_GUIDE.md](MOVEIT_GUIDE.md) for detailed documentation.
