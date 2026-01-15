# 🎯 Pick and Place Workflow Guide

Complete guide for autonomous pick and place operations combining navigation and manipulation.

## 📋 Overview

The pick and place workflow coordinates:
- **Nav2** - Autonomous navigation between waypoints
- **MoveIt2** - Intelligent arm motion planning
- **Gazebo** - Realistic physics simulation

**Complete Mission:**
1. Start at home position
2. Navigate to Point A (pick location)
3. Pick up box with arm
4. Navigate to Point B (place location)  
5. Place box
6. Return to home position

---

## 🚀 Quick Start

### Option 1: Default Workflow

```bash
# Terminal 1: Launch complete system
ros2 launch my_robot_controller pick_and_place.launch.py

# Terminal 2: Run workflow (after system is ready ~20 seconds)
source ~/ROS2_Project/install/setup.bash
ros2 run my_robot_controller pick_and_place_workflow.py
```

### Option 2: Configurable Workflow

```bash
# Terminal 1: Launch system
ros2 launch my_robot_controller pick_and_place.launch.py

# Terminal 2: Run with custom config
source ~/ROS2_Project/install/setup.bash
ros2 run my_robot_controller configurable_pick_place.py config/pick_place_config.json
```

---

## 📐 Default Waypoints

| Location | X | Y | Yaw | Description |
|----------|---|---|-----|-------------|
| **Home** | 0.0 | 0.0 | 0° | Starting position |
| **Point A (Pick)** | 2.0 | 1.0 | 0° | Box pickup location |
| **Point B (Place)** | -2.0 | -1.0 | 90° | Box placement location |

---

## 🔧 Customization

### Edit Waypoints

Edit [config/pick_place_config.json](../config/pick_place_config.json):

```json
{
  "home": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "pick_location": {
    "x": 2.5,
    "y": 1.5,
    "yaw": 0.0
  },
  "place_location": {
    "x": -3.0,
    "y": -2.0,
    "yaw": 1.57
  }
}
```

### Edit Arm Sequences

**Pick Sequence:**
```json
"pick_arm_sequence": [
  "ready",      // Move to ready position
  "extended",   // Reach to object
  "closed",     // Close gripper (grasp)
  "ready",      // Lift object
  "tucked"      // Tuck for safe transport
]
```

**Place Sequence:**
```json
"place_arm_sequence": [
  "ready",      // Position arm
  "extended",   // Extend to place location
  "open",       // Release object
  "ready",      // Retract
  "home"        // Return to home
]
```

### Adjust Timing

```json
"delays": {
  "between_steps": 2.0,       // Delay between major steps
  "arm_movement": 1.0,        // Delay after arm movements
  "gripper_action": 1.0       // Delay after gripper actions
}
```

---

## 📊 Workflow Breakdown

### Complete Sequence

```
START
  │
  ├─ STEP 0: Initialize
  │    └─ Move arm to home position
  │
  ├─ STEP 1: Navigate to Point A
  │    └─ Use Nav2 to reach pick location
  │
  ├─ STEP 2: Pick Object
  │    ├─ Open gripper
  │    ├─ Move to ready position
  │    ├─ Extend arm to object
  │    ├─ Close gripper (grasp)
  │    ├─ Lift object
  │    └─ Tuck arm for transport
  │
  ├─ STEP 3: Navigate to Point B
  │    └─ Use Nav2 to reach place location
  │
  ├─ STEP 4: Place Object
  │    ├─ Move to ready position
  │    ├─ Extend arm to place location
  │    ├─ Open gripper (release)
  │    ├─ Retract arm
  │    └─ Return to home position
  │
  ├─ STEP 5: Return Home
  │    └─ Use Nav2 to return to start
  │
  └─ FINAL: Home position
       └─ Ensure arm is at home
END (Success!)
```

---

## 🎮 Usage Examples

### Basic Usage

```bash
# Launch full system
ros2 launch my_robot_controller pick_and_place.launch.py

# Wait for system to initialize (~20 seconds)
# Look for these messages:
# - "Nav2 is ready"
# - "MoveIt initialized successfully"

# In new terminal:
ros2 run my_robot_controller pick_and_place_workflow.py
```

### With Custom Waypoints

Create `my_config.json`:
```json
{
  "home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
  "pick_location": {"x": 3.0, "y": 2.0, "yaw": 0.0},
  "place_location": {"x": -3.0, "y": -2.0, "yaw": 3.14}
}
```

Run:
```bash
ros2 run my_robot_controller configurable_pick_place.py my_config.json
```

---

## 🔍 Monitoring & Debugging

### Check System Status

```bash
# Check Nav2 status
ros2 topic echo /navigation_result

# Check arm joint states
ros2 topic echo /joint_states

# Check move_group status
ros2 node info /move_group

# Check controllers
ros2 control list_controllers
```

### Visualize in RViz

The launch file automatically opens RViz with:
- **Map** - Shows navigation map
- **Robot Model** - Visual representation
- **Global Path** - Planned navigation path
- **Local Path** - Current following path
- **MotionPlanning** - MoveIt planning visualization

---

## 🐛 Troubleshooting

### Issue: "Navigation goal rejected"

**Cause:** Goal is in obstacle or unreachable

**Solution:**
1. Check map in RViz
2. Adjust waypoint coordinates
3. Ensure map is properly loaded

```bash
# Verify map is published
ros2 topic echo /map --once
```

### Issue: "Planning failed" for arm

**Cause:** Target pose unreachable or collision detected

**Solution:**
1. Check if robot is at correct position
2. Verify object is in reach
3. Check for collisions in RViz

```bash
# Test arm independently
ros2 run my_robot_controller arm_moveit_control.py ready
```

### Issue: Workflow stops mid-execution

**Cause:** Timeout or action failure

**Solution:**
1. Check logs for specific error
2. Increase delays in config
3. Verify all nodes are running

```bash
# Check all required nodes
ros2 node list | grep -E "move_group|controller_manager|amcl"
```

### Issue: "ModuleNotFoundError: moveit"

**Solution:**
```bash
pip3 install moveit
source ~/ROS2_Project/install/setup.bash
```

---

## ⚙️ Advanced Configuration

### Multiple Pick-Place Cycles

Create loop script:

```python
#!/usr/bin/env python3
import rclpy
from configurable_pick_place import ConfigurablePickPlace

rclpy.init()
node = ConfigurablePickPlace('config/pick_place_config.json')

# Run multiple cycles
for i in range(3):
    print(f"\n=== Cycle {i+1}/3 ===")
    node.execute_workflow()
    
node.destroy_node()
rclpy.shutdown()
```

### Integration with Object Detection

Add object detection before pick:

```python
def detect_and_pick(self):
    # Your object detection code here
    object_pose = self.detect_object()
    
    # Navigate to detected object
    self.navigate_to_pose(object_pose)
    
    # Execute pick
    self.pick_sequence()
```

---

## 📈 Performance Optimization

### Speed Up Navigation

Edit [nav2_params.yaml](../config/nav2_params.yaml):

```yaml
controller_server:
  FollowPath:
    max_vel_x: 0.5  # Increase from 0.26
    max_vel_theta: 2.0  # Increase from 1.0
```

### Speed Up Arm Movements

Edit [joint_limits.yaml](../config/moveit/joint_limits.yaml):

```yaml
shoulder_joint:
  max_velocity: 2.0  # Increase from 1.0
  max_acceleration: 1.0  # Increase from 0.5
```

---

## 📚 Related Documentation

- [MOVEIT_GUIDE.md](MOVEIT_GUIDE.md) - Complete MoveIt guide
- [PROJECT_GUIDE.md](../PROJECT_GUIDE.md) - Overall project guide
- [QUICK_START.md](../QUICK_START.md) - Quick start guide

---

## ✅ Pre-Flight Checklist

Before running workflow:

- [ ] Map created and saved (`my_robot_map.yaml/pgm` exists)
- [ ] MoveIt installed (`pip3 install moveit`)
- [ ] Nav2 installed (`ros-humble-navigation2`)
- [ ] Workspace built (`colcon build`)
- [ ] Workspace sourced (`source install/setup.bash`)
- [ ] Launch file started
- [ ] All nodes running (check `ros2 node list`)
- [ ] No error messages in terminals

---

## 🎯 Success Criteria

Workflow is successful when:

✅ Robot navigates to Point A without collision  
✅ Arm successfully picks object  
✅ Object remains grasped during transport  
✅ Robot navigates to Point B safely  
✅ Object is placed correctly  
✅ Robot returns home  
✅ No errors in any terminal  

---

## 🎉 Next Steps

1. **Test basic workflow:**
   ```bash
   ros2 run my_robot_controller pick_and_place_workflow.py
   ```

2. **Customize for your environment:**
   - Adjust waypoints in config file
   - Tune navigation parameters
   - Modify arm sequences

3. **Add complexity:**
   - Multiple pick-place cycles
   - Object detection integration
   - Error recovery strategies
   - Multi-robot coordination

---

**Your mobile manipulator is now ready for autonomous pick and place operations!** 🤖📦
