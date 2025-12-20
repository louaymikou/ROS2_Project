# 🎯 QUICK REFERENCE - Autonomous Pick-and-Place Robot

## 📋 SYSTEM OVERVIEW
```
Hybrid Mobile Manipulator
├─ Base: 4-wheel differential drive (0.9m x 0.9m)
├─ Arm: 3-DOF (shoulder, elbow, rotate)
├─ Gripper: 2-finger parallel
├─ Sensor: 2D LIDAR (360°, 12m)
└─ Mission: Pick from Point A → Place at Point B → Return to A
```

---

## ⚡ QUICK START

### **Install Dependencies (Once)**
```bash
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

### **Build (After any code changes)**
```bash
cd /home/lamiae/ROS2_Project/ROS2_Project
colcon build --symlink-install
source install/setup.bash
```

### **Phase 1: Build Map (First time)**
```bash
# Terminal 1
ros2 launch my_robot_controller slam_mapping.launch.py

# Terminal 2
python3 src/my_robot_controller/keyboard_controller.py
# (Drive around slowly)

# Terminal 3 (when done)
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### **Phase 2: Run Mission**
```bash
# Terminal 1
ros2 launch my_robot_controller autonomous_mission.launch.py
# (Wait ~15 seconds for Nav2 to initialize)

# Terminal 2
ros2 run my_robot_controller mission_orchestrator.py
```

---

## 📁 FILE STRUCTURE

```
my_robot_controller/
├── action/
│   ├── PickPlace.action          # Pick/place operation
│   └── MoveArm.action             # Arm movement
├── config/
│   ├── slam_params.yaml           # SLAM configuration
│   ├── nav2_params.yaml           # Navigation parameters
│   └── my_controllers.yaml        # ros2_control config
├── description/
│   ├── robot.urdf.xacro           # Main robot (with LIDAR)
│   └── lidar.xacro                # LIDAR sensor
├── launch/
│   ├── slam_mapping.launch.py    # For building map
│   └── autonomous_mission.launch.py  # Complete system
├── scripts/
│   ├── arm_action_server.py       # Arm control
│   ├── pick_place_action_server.py   # Pick/place logic
│   └── mission_orchestrator.py    # A→B→A coordinator
└── worlds/
    └── my_world.world             # Gazebo environment
```

---

## 🔧 KEY PARAMETERS

### **Mission Waypoints** (`mission_orchestrator.py`)
```python
POINT_A = Point(x=0.0, y=5.5, z=0.5)  # Cube pickup
POINT_B = Point(x=3.0, y=0.0, z=0.5)  # Drop-off (CHANGE THIS!)
```

### **Robot Speed** (`nav2_params.yaml`)
```yaml
max_vel_x: 0.5       # Linear velocity (m/s)
max_vel_theta: 1.0   # Angular velocity (rad/s)
```

### **Robot Footprint** (`nav2_params.yaml`)
```yaml
robot_radius: 0.65   # Safety buffer (meters)
```

### **Arm Positions** (`pick_place_action_server.py`)
```python
ARM_HOME = [0.0, 0.0, 0.0]           # Safe for navigation
ARM_PICK_GRASP = [0.75, 0.75, 0.0]   # Grasping height
GRIPPER_OPEN = -0.12                 # Fully open
GRIPPER_CLOSED = 0.0                 # Grasping
```

---

## 🐛 COMMON ISSUES

| Problem | Solution |
|---------|----------|
| LIDAR not visible | `colcon build && source install/setup.bash` |
| No /scan topic | Check Gazebo LIDAR plugin loaded |
| Nav2 won't plan | Ensure /map topic publishing |
| Arm won't move | Check `ros2 control list_controllers` |
| Robot stuck | Reduce inflation_radius in nav2_params.yaml |
| Action server missing | Wait 15s after launch for initialization |

---

## 📊 USEFUL COMMANDS

### **Check Status**
```bash
ros2 node list                    # All running nodes
ros2 topic list                   # All topics
ros2 action list                  # All action servers
ros2 control list_controllers     # Hardware controllers
```

### **Monitor Topics**
```bash
ros2 topic echo /scan --once      # LIDAR data
ros2 topic echo /odom             # Robot odometry
ros2 topic echo /map --once       # SLAM map
ros2 topic hz /scan               # LIDAR frequency
```

### **Test Components**
```bash
# Test navigation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"

# Test pick
ros2 action send_goal /pick_place my_robot_controller/action/PickPlace \
  "{target_position: {x: 0.0, y: 5.5, z: 0.5}, operation: 'pick'}"

# Test arm
ros2 action send_goal /move_arm my_robot_controller/action/MoveArm \
  "{shoulder_target: 0.5, elbow_target: 0.5, gripper_rotation_target: 0.0, duration: 2.0}"
```

---

## 🎮 KEYBOARD CONTROLS (During Mapping)

```
┌─────────────────────────────┐
│  W/Z : Forward              │
│  S   : Backward             │
│  A/Q : Rotate Left          │
│  D   : Rotate Right         │
│  SPACE : Stop               │
│  Ctrl+C : Exit              │
└─────────────────────────────┘
```

---

## 📈 EXPECTED MISSION TIMELINE

```
0:00  Launch autonomous_mission.launch.py
0:15  Nav2 fully initialized (check logs)
0:15  Run mission_orchestrator.py
0:20  Robot starts navigating to Point A
0:50  Arrives at Point A, begins pick sequence
1:10  Cube grasped, arm retracted
1:15  Navigating to Point B
1:45  Arrives at Point B, begins place sequence
2:00  Cube placed, arm retracted
2:05  Navigating back to Point A
2:35  Mission complete! ✅
```

---

## 🔍 LOG MESSAGES TO WATCH FOR

### **Good Signs ✅**
```
[slam_toolbox]: Message filter subscribing to topics
[controller_server]: Controller Server has activated
[planner_server]: Planner Server has activated
[pick_place_action_server]: Pick and Place Action Server started
[mission_orchestrator]: STARTING AUTONOMOUS PICK-AND-PLACE MISSION
[mission_orchestrator]: ✅ Phase 1 complete: Cube picked
[mission_orchestrator]: 🎉 MISSION COMPLETE!
```

### **Warning Signs ⚠️**
```
[nav2]: No path found
[arm_controller]: Goal tolerance violated
[pick_place_server]: Navigation goal rejected
[slam_toolbox]: TF timeout
```

---

## 🎯 CUSTOMIZATION CHECKLIST

- [ ] Update `POINT_B` coordinates for your drop-off location
- [ ] Adjust `robot_radius` if robot collides with walls
- [ ] Tune `max_vel_x` for faster/slower navigation
- [ ] Modify `ARM_PICK_GRASP` if cube at different height
- [ ] Change `GRIPPER_OPEN` distance for larger objects

---

## 📞 HELP & DEBUGGING

**Can't build?**
→ Check all dependencies installed
→ Ensure ROS2 Humble sourced

**Robot won't navigate?**
→ Verify map exists and SLAM localized
→ Check `/tf` has map→odom→base_link chain

**Arm doesn't move?**
→ Test controllers: `ros2 control list_controllers`
→ Check joint limits in URDF

**Gripper won't grasp?**
→ Verify object within reach (~0.5m from base)
→ Check gripper opening matches object size

---

**Created:** December 19, 2025
**System:** Hybrid Mobile Manipulator with SLAM + Nav2 + ros2_control
**Mission:** Autonomous Pick-and-Place A→B→A
