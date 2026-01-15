# 🎯 Pick and Place - Quick Reference

## 🚀 Quick Start (2 Steps)

### Step 1: Launch System
```bash
ros2 launch my_robot_controller pick_and_place.launch.py
```
Wait ~20 seconds for initialization

### Step 2: Run Workflow
```bash
# New terminal
source ~/ROS2_Project/install/setup.bash
ros2 run my_robot_controller pick_and_place_workflow.py
```

---

## 📋 What Happens

1. ✅ Robot arm moves to home
2. ✅ Navigate to Point A (pick location)
3. ✅ Pick box with arm
4. ✅ Navigate to Point B (place location)
5. ✅ Place box
6. ✅ Return to home

**Total time:** ~2-3 minutes

---

## 📐 Default Locations

| Point | X | Y | Description |
|-------|---|---|-------------|
| Home | 0.0 | 0.0 | Start/End |
| Point A | 2.0 | 1.0 | Pick box |
| Point B | -2.0 | -1.0 | Place box |

---

## 🔧 Customize Waypoints

Edit: [config/pick_place_config.json](../config/pick_place_config.json)

```json
{
  "pick_location": {"x": 3.0, "y": 2.0, "yaw": 0.0},
  "place_location": {"x": -3.0, "y": -2.0, "yaw": 1.57}
}
```

Then run:
```bash
ros2 run my_robot_controller configurable_pick_place.py config/pick_place_config.json
```

---

## 🐛 Quick Fixes

**Navigation failed?**
- Check map exists: `ls src/my_robot_controller/maps/`
- Verify Nav2 ready: `ros2 node list | grep amcl`

**Arm planning failed?**
- Test arm: `ros2 run my_robot_controller arm_moveit_control.py home`
- Check MoveIt: `ros2 node list | grep move_group`

**ModuleNotFoundError?**
```bash
pip3 install moveit
```

---

## 📖 Full Documentation

[PICK_AND_PLACE_GUIDE.md](PICK_AND_PLACE_GUIDE.md) - Complete guide with troubleshooting

---

## ⚡ Alternative Launches

### Use with existing map
```bash
# Terminal 1: Launch
ros2 launch my_robot_controller pick_and_place.launch.py

# Terminal 2: Workflow
ros2 run my_robot_controller pick_and_place_workflow.py
```

### Just add workflow to running navigation
```bash
# If navigation already running:
ros2 launch my_robot_controller moveit_only.launch.py
ros2 run my_robot_controller pick_and_place_workflow.py
```

---

**🎉 Your robot is ready for autonomous pick and place!**
