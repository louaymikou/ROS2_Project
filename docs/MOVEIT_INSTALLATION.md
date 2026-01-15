# 🚀 MoveIt2 Installation & Setup Instructions

Complete installation guide for MoveIt2 arm control.

---

## ✅ Prerequisites

- ROS2 Humble installed
- Gazebo installed
- Existing workspace built and working

---

## 📦 Step 1: Install MoveIt2 Packages

```bash
# Core MoveIt2 packages
sudo apt update
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-ros-planning \
                 ros-humble-moveit-ros-planning-interface \
                 ros-humble-moveit-planners-ompl \
                 ros-humble-moveit-simple-controller-manager \
                 ros-humble-moveit-servo

# Additional dependencies
sudo apt install ros-humble-geometric-shapes \
                 ros-humble-kdl-parser \
                 ros-humble-warehouse-ros-mongo

# Python interface (important!)
sudo apt install python3-pip
pip3 install moveit
```

---

## 🔨 Step 2: Build the Workspace

```bash
cd ~/ROS2_Project

# Clean previous build (recommended)
rm -rf build/ install/ log/

# Build with MoveIt dependencies
colcon build --packages-select my_robot_controller

# Source the workspace
source install/setup.bash
```

---

## ✅ Step 3: Verify Installation

### Check MoveIt Packages

```bash
# Check if MoveIt packages are installed
ros2 pkg list | grep moveit

# Should show packages like:
# moveit
# moveit_core
# moveit_planners_ompl
# moveit_ros_planning
# etc.
```

### Check Python MoveIt

```bash
python3 -c "from moveit.planning import MoveItPy; print('✅ MoveIt Python OK')"
```

---

## 🧪 Step 4: Test the Setup

### Test 1: Launch Full System

```bash
ros2 launch my_robot_controller moveit.launch.py
```

**Expected output:**
- ✅ Gazebo opens with robot
- ✅ RViz opens with MoveIt interface
- ✅ No error messages
- ✅ Orange interactive markers visible in RViz

### Test 2: Simple Arm Movement

In a new terminal:

```bash
source ~/ROS2_Project/install/setup.bash
ros2 run my_robot_controller arm_moveit_control.py home
```

**Expected:**
- ✅ "MoveIt initialized successfully!" message
- ✅ "Planning successful!" message
- ✅ Arm moves to home position
- ✅ No errors

### Test 3: Interactive Control

```bash
source ~/ROS2_Project/install/setup.bash
ros2 run my_robot_controller interactive_arm_control.py
```

**Expected:**
- ✅ Interactive menu appears
- ✅ Can select options 1-9
- ✅ Arm responds to commands

---

## 🐛 Troubleshooting

### Issue 1: "ModuleNotFoundError: No module named 'moveit'"

**Solution:**
```bash
pip3 install moveit
# OR
pip3 install moveit-py
```

### Issue 2: "Planning failed" or "No IK solution found"

**Cause:** KDL kinematics plugin not installed

**Solution:**
```bash
sudo apt install ros-humble-kdl-kinematics-plugin
colcon build --packages-select my_robot_controller
source install/setup.bash
```

### Issue 3: Controllers not found

**Solution:**
```bash
# Make sure controllers are spawned
ros2 control list_controllers

# If arm_controller missing:
ros2 run controller_manager spawner arm_controller

# If gripper_controller missing:
ros2 run controller_manager spawner gripper_controller
```

### Issue 4: "move_group" node not starting

**Check dependencies:**
```bash
sudo apt install ros-humble-moveit-ros-move-group
```

### Issue 5: RViz crashes or no MoveIt panel

**Solution:**
```bash
sudo apt install ros-humble-moveit-ros-visualization
```

---

## 📝 Configuration Files Created

Your workspace now has these new files:

```
src/my_robot_controller/
├── config/moveit/
│   ├── my_robot.srdf                # Robot semantics
│   ├── kinematics.yaml              # IK solver config
│   ├── joint_limits.yaml            # Motion limits
│   ├── moveit_controllers.yaml      # Controller mappings
│   ├── moveit.yaml                  # General settings
│   └── ompl_planning.yaml           # Path planners
├── launch/
│   ├── moveit.launch.py             # Full launch
│   └── moveit_only.launch.py        # MoveIt only
├── nodes/controllers/
│   ├── arm_moveit_control.py        # Simple control
│   └── interactive_arm_control.py   # Interactive menu
└── docs/
    ├── MOVEIT_GUIDE.md              # Full guide
    └── MOVEIT_QUICK_START.md        # Quick reference
```

---

## 🎯 Quick Test Sequence

Complete verification in 5 steps:

```bash
# Step 1: Install
sudo apt install ros-humble-moveit ros-humble-moveit-planners-ompl
pip3 install moveit

# Step 2: Build
cd ~/ROS2_Project
colcon build
source install/setup.bash

# Step 3: Launch (Terminal 1)
ros2 launch my_robot_controller moveit.launch.py

# Step 4: Test home position (Terminal 2)
source ~/ROS2_Project/install/setup.bash
ros2 run my_robot_controller arm_moveit_control.py home

# Step 5: Run demo (Terminal 2)
ros2 run my_robot_controller arm_moveit_control.py demo
```

If all 5 steps work: ✅ **Installation successful!**

---

## 🎓 Next Steps

1. **Read the guides:**
   - [MOVEIT_QUICK_START.md](MOVEIT_QUICK_START.md) - Quick commands
   - [MOVEIT_GUIDE.md](MOVEIT_GUIDE.md) - Complete documentation

2. **Try predefined poses:**
   ```bash
   ros2 run my_robot_controller arm_moveit_control.py extended
   ros2 run my_robot_controller arm_moveit_control.py tucked
   ros2 run my_robot_controller arm_moveit_control.py ready
   ```

3. **Experiment with gripper:**
   ```bash
   ros2 run my_robot_controller arm_moveit_control.py open
   ros2 run my_robot_controller arm_moveit_control.py close
   ```

4. **Use interactive control:**
   ```bash
   ros2 run my_robot_controller interactive_arm_control.py
   ```

5. **Combine with navigation:**
   ```bash
   # Terminal 1
   ros2 launch my_robot_controller navigation.launch.py
   
   # Terminal 2
   ros2 launch my_robot_controller moveit_only.launch.py
   
   # Terminal 3
   ros2 run my_robot_controller auto_navigator.py demo
   
   # Terminal 4
   ros2 run my_robot_controller arm_moveit_control.py demo
   ```

---

## ✅ Success Checklist

- [ ] MoveIt packages installed
- [ ] Python moveit module installed
- [ ] Workspace builds without errors
- [ ] moveit.launch.py starts successfully
- [ ] RViz shows MoveIt interface
- [ ] arm_moveit_control.py works
- [ ] interactive_arm_control.py works
- [ ] Demo completes successfully
- [ ] No error messages in any terminal

---

## 🆘 Still Having Issues?

### Get System Info

```bash
# ROS2 version
printenv ROS_DISTRO

# MoveIt version
ros2 pkg xml moveit | grep version

# Python packages
pip3 list | grep moveit
```

### Check Logs

```bash
# View move_group logs
ros2 run rqt_console rqt_console

# Or check terminal output carefully
```

### Common Error Messages

| Error | Solution |
|-------|----------|
| "No module named 'moveit'" | `pip3 install moveit` |
| "Planning plugin not found" | `sudo apt install ros-humble-moveit-planners-ompl` |
| "IK solver not found" | `sudo apt install ros-humble-kdl-kinematics-plugin` |
| "Controller not found" | Check `ros2 control list_controllers` |
| "move_group not found" | `sudo apt install ros-humble-moveit-ros-move-group` |

---

## 📚 Additional Resources

- [Official MoveIt2 Tutorial](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
- [ROS2 Control Documentation](https://control.ros.org/humble/index.html)
- [OMPL Planners](https://ompl.kavrakilab.org/planners.html)

---

**🎉 Installation Complete! Your arm is ready for advanced motion planning with MoveIt2!**
