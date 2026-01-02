# ROS2 Robot Project - SLAM & Navigation Guide

## Prerequisites

### Install Required Dependencies

Before starting, install the necessary Nav2 and SLAM packages:

```bash
# Nav2 Simple Commander API for programmatic control
sudo apt install ros-humble-nav2-simple-commander

# TF transformations for coordinate frame conversions
sudo apt install ros-humble-tf-transformations

# Python transforms3d library for quaternion calculations
sudo apt install python3-transforms3d

# SLAM Toolbox for mapping
sudo apt install ros-humble-slam-toolbox

# Nav2 stack
sudo apt install ros-humble-nav2-bringup
```

---

## Project Setup

### Build the Project

```bash
cd /home/wayay/ROS_PROJECT
colcon build
source install/setup.bash
```

**Note:** Always source the workspace after building!

---

## Robot Simulation (Manual Control)

### Option 1: Launch with PS4 Controller (All-in-One)
```bash
colcon build
source install/setup.bash
ros2 launch my_robot_controller launch_sim_with_ps4.launch.py
```

### Option 2: Launch with QWERTY Keyboard (All-in-One)
```bash
colcon build
source install/setup.bash
ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py
```

### Option 3: Launch Simulation and Keyboard Controller Separately

**Terminal 1:**
```bash
colcon build
source install/setup.bash
ros2 launch my_robot_controller launch_sim.launch.py
```

**Terminal 2:**
```bash
source install/setup.bash
python3 /home/wayay/ROS_PROJECT/src/my_robot_controller/keyboard_controller.py
```

### Option 4: Launch with Manual Teleop Control

**Terminal 1:**
```bash
colcon build
source install/setup.bash
ros2 launch my_robot_controller launch_sim.launch.py
```

**Terminal 2:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

**Terminal 3 (optional - arm control):**
```bash
python3 /home/wayay/ROS_PROJECT/src/my_robot_controller/simple_arm_control.py
```

---

## SLAM (Mapping)

### Step 1: Build and Source the Workspace

```bash
cd /home/wayay/ROS_PROJECT
colcon build
source install/setup.bash
```

### Step 2: Launch SLAM with Robot Simulation

**Terminal 1:**
```bash
cd /home/wayay/ROS_PROJECT
source install/setup.bash
ros2 launch my_robot_controller slam_launch.py use_sim_time:=true
```

This will:
- Start the robot simulation in Gazebo
- Launch SLAM Toolbox for mapping

### Step 3: Visualize in RViz

**Terminal 2:**
```bash
ros2 run rviz2 rviz2
```

**RViz Configuration:**
1. Set **Fixed Frame** to `map`
2. Add displays:
   - **Map** (Topic: `/map`)
   - **LaserScan** (Topic: `/scan`)
   - **RobotModel**
   - **TF**

### Step 4: Drive the Robot to Create the Map

**Terminal 3:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

**Controls:**
- `w`: Move forward
- `x`: Move backward
- `a`: Turn left
- `d`: Turn right
- `s`: Stop

Drive around the entire environment slowly to build a complete map.

### Step 5: Save the Map

When satisfied with the map, save it:

**Terminal 4:**
```bash
cd /home/wayay/ROS_PROJECT
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

This creates:
- `maps/my_map.pgm` - The map image
- `maps/my_map.yaml` - Map metadata

**To stop SLAM:** Press `Ctrl+C` in Terminal 1 to stop all SLAM processes.

### Troubleshooting SLAM

If you see warnings like "Failed to compute odom pose":

**Run the diagnostic script:**
```bash
./diagnose_slam.sh
```

This will check:
- If /odom and /scan topics are publishing
- If TF transforms are available
- Which nodes are running

**Common fixes:**
1. Make sure the robot simulation is fully loaded (wait 5-10 seconds after launch)
2. Check that odometry is being published: `ros2 topic echo /odom`
3. Check that scan data is being published: `ros2 topic echo /scan`
4. Verify TF transforms: `ros2 run tf2_ros tf2_echo odom base_link`
5. Rebuild with clean: `rm -rf build/ install/ && colcon build --symlink-install`

If SLAM is not working properly:
1. Rebuild the project: `colcon build --symlink-install`
2. Source the workspace: `source install/setup.bash`
3. Relaunch SLAM

---

## Autonomous Navigation

### Method 1: Using Nav2 with Manual Goal Setting (RViz)

#### Terminal 1: Launch Navigation Stack

```bash
cd /home/wayay/ROS_PROJECT
source install/setup.bash
ros2 launch my_robot_controller navigation_launch.py use_sim_time:=true map:=/home/wayay/ROS_PROJECT/maps/my_map.yaml
```

#### Terminal 2: Launch RViz

```bash
ros2 run rviz2 rviz2
```

**RViz Configuration:**
1. Set **Fixed Frame** to `odom` (temporarily, will change to `map` after setting initial pose)
2. Add displays:
   - **Map** (Topic: `/map`)
   - **Global Costmap** (Topic: `/global_costmap/costmap`)
   - **Local Costmap** (Topic: `/local_costmap/costmap`)
   - **Global Plan** (Topic: `/plan`)
   - **Local Plan** (Topic: `/local_plan`)
   - **Particle Cloud** (Topic: `/particle_cloud`)
   - **LaserScan** (Topic: `/scan`)

**IMPORTANT - Set Initial Pose (Required!):**

You'll see errors like "Invalid frame ID 'map'" until you set the initial pose. This is normal.

1. Click **"2D Pose Estimate"** button in RViz toolbar
2. Click and drag on the map where your robot is actually located
3. The arrow direction should match the robot's orientation
4. After setting, the `map` frame will appear and errors will stop
5. Change Fixed Frame from `odom` to `map`

**Send Navigation Goal:**
1. Click **"2D Goal Pose"** button
2. Click and drag on the map where you want the robot to go
3. The robot will plan a path and navigate autonomously

**Note:** If you see "Timed out waiting for transform from base_link to map" errors before setting the initial pose, this is expected. The map frame doesn't exist until AMCL initializes with an initial pose.

---

### Method 2: Programmatic Navigation (Python Script)

#### Terminal 1: Launch Navigation Stack

```bash
cd /home/wayay/ROS_PROJECT
source install/setup.bash
ros2 launch my_robot_controller navigation_launch.py use_sim_time:=true map:=/home/wayay/ROS_PROJECT/maps/my_map.yaml
```

#### Terminal 2: Run Autonomous Navigation Script

```bash
cd /home/wayay/ROS_PROJECT
source install/setup.bash
python3 src/my_robot_controller/autonomous_navigation.py
```

**What the script does:**
- Sets the initial robot pose
- Navigates to a single goal position
- Follows a sequence of waypoints

**Customize waypoints:**
Edit [src/my_robot_controller/autonomous_navigation.py](src/my_robot_controller/autonomous_navigation.py) and modify the coordinates:

```python
# Single goal example
goal_pose = create_pose_stamped(nav, 3.5, 1.0, 1.57)

# Waypoint example
goal_pose1 = create_pose_stamped(nav, 3.5, 1.5, 1.57)
goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, 1.57)
```

**Orientation values (radians):**
- `0.0` = East (0°)
- `1.57` = North (90°)
- `3.14` = West (180°)
- `-1.57` = South (-90°)

---

## Troubleshooting

### Common Warnings (Safe to Ignore)

**TF_OLD_DATA warnings:**
```
Warning: TF_OLD_DATA ignoring data from the past for frame...
```
These are timing warnings in simulation and don't affect navigation. Safe to ignore.

**GLSL Shader Errors:**
```
[ERROR] active samplers with a different type refer to the same texture image unit
```
Graphics driver warning in RViz. Usually doesn't affect functionality. If displays don't work, try:
```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 run rviz2 rviz2
```

**"Invalid frame ID 'map'" before setting initial pose:**
This is expected! The `map` frame doesn't exist until you publish an initial pose. Fix by:
```bash
# Publish initial pose to create map frame
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
Then in RViz, change Fixed Frame to `map` and use "2D Pose Estimate" to refine the robot's position.

### Robot Not Moving with 2D Goal Pose

If the robot doesn't move when you set a goal in RViz, check that Nav2 is publishing to the correct topic:

```bash
# Check if controller_server is publishing to the robot's cmd_vel topic
ros2 topic info /diff_cont/cmd_vel_unstamped
```

The robot's diff_drive_controller listens to `/diff_cont/cmd_vel_unstamped`. Our custom [src/my_robot_controller/config/nav2_params.yaml](src/my_robot_controller/config/nav2_params.yaml) configures Nav2 to publish to this topic via the `cmd_vel_topic` parameter in the `controller_server` section.

### RViz Issues

```bash
# List all topics
ros2 topic list

# Echo a specific topic
ros2 topic echo /scan
ros2 topic echo /map
```

### Check ROS2 Nodes

```bash
ros2 node list
```

### Check TF Transformations

```bash
ros2 run tf2_ros tf2_echo map base_link
```

### View TF Tree

```bash
ros2 run tf2_tools view_frames
```

### Monitor Navigation Feedback

```bash
ros2 topic echo /navigate_to_pose/_action/feedback
```

---

## Quick Reference Commands

### Build Project
```bash
cd /home/wayay/ROS_PROJECT && colcon build && source install/setup.bash
```

### Launch SLAM Mapping
```bash
ros2 launch my_robot_controller slam_launch.py use_sim_time:=true
```

### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

### Launch Navigation
```bash
ros2 launch my_robot_controller navigation_launch.py use_sim_time:=true map:=/home/wayay/ROS_PROJECT/maps/my_map.yaml
```

### Run Autonomous Navigation
```bash
python3 src/my_robot_controller/autonomous_navigation.py
```

### Teleop Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

---

## Project Structure

```
ROS_PROJECT/
├── src/
│   └── my_robot_controller/
│       ├── launch/
│       │   ├── slam_launch.py              # SLAM mapping launch file
│       │   ├── navigation_launch.py        # Nav2 navigation launch file
│       │   ├── launch_sim.launch.py        # Robot simulation
│       │   ├── launch_sim_with_ps4.launch.py
│       │   └── launch_sim_with_keyboard.launch.py
│       ├── autonomous_navigation.py        # Python navigation script
│       ├── keyboard_controller.py
│       ├── ps4_controller.py
│       └── simple_arm_control.py
├── maps/                                   # Saved SLAM maps
│   ├── my_map.pgm
│   └── my_map.yaml
├── config/                                 # Configuration files
└── README.md                               # This file
```

---

## Workflow Summary

1. **Build** → `colcon build && source install/setup.bash`
2. **Map** → Launch SLAM, drive around, save map
3. **Navigate** → Launch Nav2 with map, set initial pose, send goals
4. **Automate** → Run Python script for programmatic control
