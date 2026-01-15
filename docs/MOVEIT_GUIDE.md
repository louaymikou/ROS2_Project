# 🦾 MoveIt2 Arm Control Guide

Complete guide for controlling the robotic arm using MoveIt2 motion planning framework.

## 📋 Table of Contents
1. [Installation](#installation)
2. [Quick Start](#quick-start)
3. [Using MoveIt with Gazebo](#using-moveit-with-gazebo)
4. [Control Scripts](#control-scripts)
5. [Predefined Poses](#predefined-poses)
6. [Advanced Usage](#advanced-usage)
7. [Troubleshooting](#troubleshooting)

---

## 🔧 Installation

### Install MoveIt2 and Dependencies

```bash
# Install MoveIt2 for ROS2 Humble
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-ros-planning \
                 ros-humble-moveit-ros-planning-interface \
                 ros-humble-moveit-planners-ompl \
                 ros-humble-moveit-simple-controller-manager \
                 ros-humble-moveit-servo \
                 ros-humble-warehouse-ros-mongo

# Install additional packages
sudo apt install ros-humble-geometric-shapes \
                 ros-humble-kdl-parser
```

### Build the Workspace

```bash
cd ~/ROS2_Project
colcon build --packages-select my_robot_controller
source install/setup.bash
```

---

## 🚀 Quick Start

### Option 1: Full Launch (Gazebo + MoveIt)

Start everything in one launch file:

```bash
ros2 launch my_robot_controller moveit.launch.py
```

This launches:
- ✅ Gazebo simulation
- ✅ Robot spawning
- ✅ Controllers (diff_drive, arm, gripper)
- ✅ MoveIt Move Group
- ✅ RViz with MoveIt interface

### Option 2: Add MoveIt to Existing Simulation

If you already have SLAM or Navigation running:

**Terminal 1** - Start your simulation:
```bash
ros2 launch my_robot_controller slam_mapping.launch.py
# OR
ros2 launch my_robot_controller navigation.launch.py
```

**Terminal 2** - Add MoveIt:
```bash
ros2 launch my_robot_controller moveit_only.launch.py
```

---

## 🎮 Control Scripts

### Simple Command-Line Control

Control the arm with single commands:

```bash
# Arm poses
ros2 run my_robot_controller arm_moveit_control.py home
ros2 run my_robot_controller arm_moveit_control.py extended
ros2 run my_robot_controller arm_moveit_control.py tucked
ros2 run my_robot_controller arm_moveit_control.py ready

# Gripper control
ros2 run my_robot_controller arm_moveit_control.py open
ros2 run my_robot_controller arm_moveit_control.py close
ros2 run my_robot_controller arm_moveit_control.py half_open

# Run pick & place demo
ros2 run my_robot_controller arm_moveit_control.py demo
```

### Interactive Menu Control

Launch interactive control with menu:

```bash
ros2 run my_robot_controller interactive_arm_control.py
```

**Interactive Menu:**
```
           🦾 ARM CONTROL MENU
============================================================

📋 PREDEFINED POSES:
  1. Home position
  2. Extended forward
  3. Tucked (compact)
  4. Ready for pickup

🤏 GRIPPER CONTROL:
  5. Open gripper
  6. Close gripper
  7. Half open gripper

🎯 CUSTOM CONTROL:
  8. Set custom joint angles
  9. Execute pick & place demo

  0. Exit
============================================================
```

---

## 📐 Predefined Poses

### ARM Poses

| Pose Name | Description | Joint Values (deg) |
|-----------|-------------|-------------------|
| **home** | Default rest position | Shoulder: 0°, Elbow: 0°, Wrist: 0° |
| **extended** | Arm extended forward | Shoulder: 0°, Elbow: 90°, Wrist: 0° |
| **tucked** | Compact storage pose | Shoulder: -69°, Elbow: -115°, Wrist: 0° |
| **ready** | Ready to pick objects | Shoulder: 29°, Elbow: 46°, Wrist: 0° |

### GRIPPER Poses

| Pose Name | Description | Opening Width |
|-----------|-------------|---------------|
| **open** | Fully open | -0.5m each finger |
| **closed** | Fully closed | 0.0m |
| **half_open** | Partially open | -0.25m each finger |

---

## 🔬 Advanced Usage

### Custom Joint Positions (Python)

```python
#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import math

rclpy.init()
node = rclpy.create_node('custom_arm_control')

# Initialize MoveIt
moveit = MoveItPy(node=node)
arm = moveit.get_planning_component("arm")

# Set custom joint angles (in radians)
robot_state = RobotState(moveit.get_robot_model())
joint_values = [
    math.radians(30),   # shoulder_joint
    math.radians(45),   # elbow_joint
    math.radians(90)    # gripper_rotate_joint
]
robot_state.set_joint_group_positions("arm", joint_values)

# Plan and execute
arm.set_goal_state(robot_state=robot_state)
plan = arm.plan()
if plan:
    moveit.execute(plan.trajectory, controllers=[])
    print("✅ Movement completed!")

rclpy.shutdown()
```

### Cartesian Path Planning

```python
# Move end effector in a straight line
from geometry_msgs.msg import Pose

target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.8

# Plan cartesian path
waypoints = [target_pose]
(plan, fraction) = arm.compute_cartesian_path(
    waypoints,
    0.01,  # eef_step
    0.0    # jump_threshold
)

if fraction > 0.9:  # 90% of path planned
    moveit.execute(plan, controllers=[])
```

---

## 🎯 Using MoveIt in RViz

### Planning with Interactive Markers

1. **Launch MoveIt with RViz:**
   ```bash
   ros2 launch my_robot_controller moveit.launch.py
   ```

2. **In RViz:**
   - You'll see orange interactive markers on the robot
   - Drag the markers to set target pose
   - Click **"Plan"** button to generate trajectory
   - Click **"Execute"** to move the robot

3. **Available Displays:**
   - **MotionPlanning** - Main MoveIt interface
   - **RobotModel** - Visual representation
   - **PlanningScene** - Collision objects
   - **Trajectory** - Planned path visualization

### Adding Collision Objects

```python
# In your Python script
from moveit.planning import PlanningSceneInterface

scene = PlanningSceneInterface("my_robot")

# Add a box obstacle
box_pose = PoseStamped()
box_pose.header.frame_id = "base_link"
box_pose.pose.position.x = 0.5
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.5

scene.add_box("obstacle1", box_pose, size=(0.1, 0.1, 0.1))
```

---

## 📊 MoveIt Configuration Files

### File Structure

```
config/moveit/
├── my_robot.srdf                # Semantic robot description
├── kinematics.yaml              # IK solver configuration
├── joint_limits.yaml            # Velocity/acceleration limits
├── ompl_planning.yaml           # Motion planning algorithms
├── moveit_controllers.yaml      # Controller mappings
└── moveit.yaml                  # General MoveIt settings
```

### Planning Groups

| Group Name | Joints | Purpose |
|------------|--------|---------|
| **arm** | shoulder, elbow, gripper_rotate | Arm motion planning |
| **gripper** | gripper_left, gripper_right | Gripper control |
| **manipulator** | arm + gripper | Full system |

### Available Planners

- **RRTConnect** (default) - Fast, good for simple paths
- **RRTstar** - Optimal paths, slower
- **PRM** - Multi-query planner
- **BKPIECE** - Grid-based planner
- **EST** - Expansive Space Trees

---

## ⚙️ Integration with Navigation

### Coordinated Mobile Manipulation

Combine MoveIt arm control with Nav2 navigation:

**Terminal 1** - Start Navigation:
```bash
ros2 launch my_robot_controller navigation.launch.py
```

**Terminal 2** - Add MoveIt:
```bash
ros2 launch my_robot_controller moveit_only.launch.py
```

**Terminal 3** - Navigate to object:
```bash
ros2 run my_robot_controller auto_navigator.py demo
```

**Terminal 4** - Pick object with arm:
```bash
ros2 run my_robot_controller arm_moveit_control.py demo
```

---

## 🔍 Troubleshooting

### Issue: "Planning failed"

**Possible causes:**
- Target pose is unreachable
- Collision detected
- Joint limits exceeded

**Solutions:**
```bash
# Check if controllers are running
ros2 control list_controllers

# Verify joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames
```

### Issue: "Controller not found"

**Solution:**
```bash
# Ensure controllers are spawned
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller
```

### Issue: "MoveIt not connecting"

**Solution:**
```bash
# Check move_group node is running
ros2 node list | grep move_group

# Check if action server is available
ros2 action list | grep MoveGroup
```

### Issue: Arm moves erratically

**Causes:**
- Velocity/acceleration limits too high
- Controller gains not tuned

**Solution:**
Edit [config/moveit/joint_limits.yaml](../config/moveit/joint_limits.yaml):
```yaml
shoulder_joint:
  max_velocity: 0.5      # Reduce from 1.0
  max_acceleration: 0.3  # Reduce from 0.5
```

---

## 📈 Performance Tips

### Optimize Planning

```yaml
# In ompl_planning.yaml
arm:
  default_planner_config: RRTConnect  # Fastest
  longest_valid_segment_fraction: 0.01  # Increase for faster planning
```

### Speed Up Execution

```yaml
# In joint_limits.yaml
shoulder_joint:
  max_velocity: 2.0          # Increase (be careful!)
  max_acceleration: 1.0
```

### Reduce Planning Time

```yaml
# In kinematics.yaml
arm:
  kinematics_solver_timeout: 0.02  # Reduce from 0.05
  kinematics_solver_attempts: 2    # Reduce from 3
```

---

## 🎓 Learning Resources

### Predefined Sequences

Study the pick & place demo code in [arm_moveit_control.py](../nodes/controllers/arm_moveit_control.py) to learn:
- Sequential motion planning
- Gripper coordination
- Error handling
- Timing between movements

### Custom Applications

Create your own manipulation tasks:
1. Define task waypoints
2. Use predefined poses as starting points
3. Add gripper actions between arm movements
4. Test incrementally

---

## 🔗 Related Documentation

- [PROJECT_GUIDE.md](../PROJECT_GUIDE.md) - Overall project guide
- [QUICK_START.md](../QUICK_START.md) - Quick commands
- [Official MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)

---

## ✅ Summary

**MoveIt2 Integration Complete!**

You now have:
- ✅ Full MoveIt2 configuration
- ✅ Predefined poses for common tasks
- ✅ Simple command-line control
- ✅ Interactive menu interface
- ✅ Pick & place demonstrations
- ✅ RViz visualization
- ✅ Integration with navigation

**Ready to control your arm!** 🦾
