# 🤖 ROS2 Mobile Manipulator Robot - Complete Project Specification

**Version:** 2.0  
**Date:** January 2, 2026  
**Author:** Lamiaehadi  
**Target ROS Version:** ROS2 Humble  
**Target OS:** Ubuntu 22.04 LTS  

---

## 📋 EXECUTIVE SUMMARY

This document contains the **complete specification** for rebuilding a ROS2 mobile manipulator robot project from scratch. The robot features:
- **4-wheel differential drive** mobile base
- **2-DOF robotic arm** with gripper
- **IMU + Wheel Encoder fusion** for improved odometry
- **2D LIDAR** for SLAM mapping
- **Full Nav2 integration** for autonomous navigation
- **Multiple control modes** (keyboard, PS4 controller)
- **Automated mapping** capabilities

Use this document as a comprehensive instruction set for an AI coding agent to rebuild the entire project systematically.

---

## 🎯 PROJECT GOALS

### Primary Objectives
1. ✅ Create a fully functional mobile manipulator in Gazebo simulation
2. ✅ Implement sensor fusion (IMU + Wheel Encoders) using `robot_localization`
3. ✅ Enable SLAM mapping with `slam_toolbox`
4. ✅ Integrate Nav2 for autonomous navigation
5. ✅ Provide multiple teleoperation modes
6. ✅ Create automated mapping scripts

### Success Criteria
- Robot spawns correctly in Gazebo without errors
- EKF publishes fused odometry at ~50Hz
- SLAM creates accurate maps without drift
- Nav2 successfully navigates to waypoints
- All controllers work reliably
- Documentation is complete and clear

---

## 📦 SYSTEM REQUIREMENTS

### Operating System
- **Ubuntu 22.04 LTS (Jammy Jellyfish)** - Required
- Minimum 8GB RAM (16GB recommended)
- 20GB free disk space
- 64-bit processor

### Software Dependencies

#### Core ROS2 Packages
```bash
# ROS2 Humble Desktop (full installation)
ros-humble-desktop

# Simulation
ros-humble-gazebo-ros-pkgs

# Navigation
ros-humble-navigation2
ros-humble-nav2-bringup

# SLAM
ros-humble-slam-toolbox

# Sensor Fusion
ros-humble-robot-localization

# Controllers
ros-humble-ros2-controllers
ros-humble-ros2-control
ros-humble-gazebo-ros2-control
ros-humble-diff-drive-controller
ros-humble-joint-state-broadcaster
ros-humble-joint-trajectory-controller

# Utilities
ros-humble-xacro
ros-humble-robot-state-publisher
ros-humble-tf2-tools
ros-humble-twist-mux

# Build tools
python3-colcon-common-extensions
```

#### Python Dependencies (none beyond standard ROS2)

---

## 🏗️ PROJECT STRUCTURE

```
ros2_mobile_robot/                    # Root workspace
├── .github/
│   └── workflows/
│       └── build.yml                 # CI/CD (optional)
├── src/
│   └── my_robot_controller/          # Main package
│       ├── package.xml               # Package manifest
│       ├── CMakeLists.txt            # Build configuration
│       ├── config/                   # Configuration files
│       │   ├── ekf_params.yaml       # EKF sensor fusion config
│       │   ├── my_controllers.yaml   # Robot controllers config
│       │   ├── nav2_params.yaml      # Navigation stack config
│       │   ├── slam_params.yaml      # SLAM configuration
│       │   └── slam_stable.yaml      # Alternative SLAM config
│       ├── description/              # Robot URDF/Xacro files
│       │   ├── robot.urdf.xacro      # Main robot description
│       │   ├── imu.xacro             # IMU sensor definition
│       │   └── lidar.xacro           # LIDAR sensor definition
│       ├── launch/                   # Launch files
│       │   ├── launch_sim.launch.py            # Basic simulation
│       │   ├── launch_sim_with_keyboard.launch.py  # Sim + keyboard
│       │   ├── launch_sim_with_ps4.launch.py   # Sim + PS4 controller
│       │   ├── robot_localization.launch.py    # EKF only
│       │   ├── slam_mapping.launch.py          # SLAM with EKF
│       │   └── launch_mapping.launch.py        # Alternative mapping
│       ├── worlds/                   # Gazebo world files
│       │   └── my_world.world        # Custom simulation environment
│       ├── models/                   # Gazebo models (optional)
│       └── nodes/                    # Python nodes
│           ├── controllers/
│           │   ├── keyboard_controller.py      # Keyboard teleoperation
│           │   └── ps4_controller.py           # PS4 controller support
│           ├── mappers/
│           │   ├── auto_mapper.py              # Autonomous mapping
│           │   ├── auto_mapper_detailed.py     # Detailed mapping
│           │   └── wall_follower_mapper.py     # Wall-following mapper
│           ├── navigation/
│           │   ├── navigate_to_package.py      # Navigate to objects
│           │   ├── set_initial_pose.py         # Set Nav2 pose
│           │   └── backward_to_wall.py         # Backup navigation
│           └── compare_odometry.py             # Compare odom sources
├── maps/                             # Generated maps
│   ├── my_robot_map.pgm              # Map image
│   └── my_robot_map.yaml             # Map metadata
├── docs/                             # Documentation
│   ├── INSTALLATION_GUIDE.md
│   ├── IMU_INTEGRATION_GUIDE.md
│   ├── IMU_ARCHITECTURE.md
│   ├── EKF_TUNING_GUIDE.md
│   └── IMU_IMPROVEMENTS.md
├── README.md                         # Main documentation
├── QUICKSTART_IMU.md                 # Quick start guide
├── USEFUL_COMMANDS.md                # Command reference
├── .gitignore
└── LICENSE
```

---

## 🔧 DETAILED COMPONENT SPECIFICATIONS

### 1. ROBOT DESCRIPTION (URDF/Xacro)

#### 1.1 Main Robot (`description/robot.urdf.xacro`)

**Chassis Specifications:**
- **Type:** Rectangular box
- **Dimensions:** 0.9m × 0.9m × 0.15m
- **Mass:** 5.0 kg
- **Color:** Blue (RGB: 0.1, 0.3, 0.6)
- **Position:** Base at z=0.075m

**Wheel Specifications:**
- **Type:** Cylindrical
- **Radius:** 0.15m
- **Width:** 0.12m
- **Mass per wheel:** 0.2 kg
- **Configuration:** 4-wheel differential drive (front/rear, left/right)
- **Wheel separation:** 1.14m (left to right)
- **Wheelbase:** 0.6m (front to rear)
- **Color:** Dark grey (RGB: 0.15, 0.15, 0.15)

**Wheel Positions:**
- Front Left: (x=0.3, y=0.525, z=0)
- Front Right: (x=0.3, y=-0.525, z=0)
- Rear Left: (x=-0.3, y=0.525, z=0)
- Rear Right: (x=-0.3, y=-0.525, z=0)

**Robotic Arm Specifications:**
- **Joint 1 (Shoulder):** Revolute, range [-1.57, 1.57] rad
  - Link: 0.15 × 0.15 × 0.6m box
  - Mass: 0.1 kg
  - Color: Orange (RGB: 0.9, 0.4, 0.1)
  
- **Joint 2 (Elbow):** Revolute, range [-1.57, 1.57] rad
  - Link: 0.1 × 0.1 × 0.5m box
  - Mass: 0.05 kg
  
- **Joint 3 (Gripper Rotation):** Revolute, continuous
  - Platform: 0.1 × 0.1 × 0.05m box
  - Mass: 0.02 kg

**Gripper Specifications:**
- **Type:** Parallel jaw gripper
- **Fingers:** 2 prismatic joints
- **Joint range:** [0, 0.04] m
- **Finger dimensions:** 0.02 × 0.01 × 0.1m each
- **Mass per finger:** 0.01 kg
- **Color:** Grey metal (RGB: 0.3, 0.3, 0.35)

**Gazebo Plugins Required:**
```xml
<plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
  <robot_param>robot_description</robot_param>
  <robot_param_node>robot_state_publisher</robot_param_node>
  <parameters>$(find my_robot_controller)/config/my_controllers.yaml</parameters>
</plugin>
```

**Required Includes:**
```xml
<xacro:include filename="imu.xacro"/>
<xacro:include filename="lidar.xacro"/>
```

#### 1.2 IMU Sensor (`description/imu.xacro`)

**Physical Specifications:**
- **Type:** Box
- **Dimensions:** 0.05 × 0.05 × 0.02m
- **Mass:** 0.015 kg
- **Mount position:** (0.3, 0, 0.15) on base_link
- **Color:** Green (RGB: 0, 1, 0)

**Sensor Characteristics:**
- **Update rate:** 100 Hz
- **Topic:** `/imu/data`
- **Message type:** `sensor_msgs/Imu`
- **Frame:** `imu_link`

**Noise Model:**
```xml
<angular_velocity>
  <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
  <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
  <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
</angular_velocity>
<linear_acceleration>
  <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
  <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
  <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
</linear_acceleration>
```

**Gazebo Plugin:**
```xml
<plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
  <ros>
    <namespace></namespace>
    <remapping>~/out:=imu/data</remapping>
  </ros>
  <initial_orientation_as_reference>false</initial_orientation_as_reference>
</plugin>
```

#### 1.3 LIDAR Sensor (`description/lidar.xacro`)

**Physical Specifications:**
- **Type:** Cylinder
- **Radius:** 0.05m
- **Height:** 0.04m
- **Mass:** 0.125 kg
- **Mount position:** (0, 0, 0.17) on base_link
- **Color:** Red (RGB: 1, 0, 0)

**Sensor Characteristics:**
- **Type:** 2D Laser Scanner (ray sensor)
- **Update rate:** 20 Hz
- **Samples:** 720 rays
- **Angular range:** -π to +π (360°)
- **Resolution:** 0.5°
- **Min range:** 0.12m
- **Max range:** 12.0m
- **Topic:** `/scan`
- **Message type:** `sensor_msgs/LaserScan`

**Range Noise:**
```xml
<range>
  <min>0.12</min>
  <max>12.0</max>
  <resolution>0.01</resolution>
</range>
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

**Gazebo Plugin:**
```xml
<plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
  <ros>
    <remapping>~/out:=scan</remapping>
  </ros>
  <output_type>sensor_msgs/LaserScan</output_type>
  <frame_name>laser_frame</frame_name>
</plugin>
```

---

### 2. CONTROLLER CONFIGURATION

#### 2.1 Controller Manager (`config/my_controllers.yaml`)

**Update Rate:** 30 Hz

**Controllers to Spawn:**
1. `diff_cont` - Differential Drive Controller
2. `joint_broad` - Joint State Broadcaster
3. `arm_controller` - Arm Trajectory Controller
4. `gripper_controller` - Gripper Trajectory Controller

#### 2.2 Differential Drive Controller

```yaml
diff_cont:
  ros__parameters:
    left_wheel_names: ["front_left_joint", "rear_left_joint"]
    right_wheel_names: ["front_right_joint", "rear_right_joint"]
    wheel_separation: 1.14        # Distance between left and right wheels
    wheel_radius: 0.15            # Wheel radius in meters
    use_stamped_vel: false
    publish_rate: 50.0            # Odometry publication rate
    base_frame_id: base_link
    odom_frame_id: odom
    enable_odom_tf: true          # Publish odom->base_link transform
    
    # Covariance matrices (diagonal)
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.1]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.1]
    # Note: High covariance (0.1) on yaw indicates uncertainty in rotation
    
    open_loop: false              # Use wheel encoder feedback
    reference_timeout: 1.0
    state_publish_period: 0.02
```

**Published Topics:**
- `/diff_cont/odom` - Raw wheel odometry
- `/diff_cont/cmd_vel_unstamped` - Velocity commands (subscribed)

#### 2.3 Arm Controller

```yaml
arm_controller:
  ros__parameters:
    joints:
      - shoulder_joint
      - elbow_joint
      - gripper_rotate_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

**Topic:** `/arm_controller/joint_trajectory`

#### 2.4 Gripper Controller

```yaml
gripper_controller:
  ros__parameters:
    joints:
      - gripper_left_joint
      - gripper_right_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

**Topic:** `/gripper_controller/joint_trajectory`

---

### 3. SENSOR FUSION CONFIGURATION

#### 3.1 EKF Node (`config/ekf_params.yaml`)

**Purpose:** Fuse wheel odometry and IMU data for improved pose estimation

**Key Parameters:**
```yaml
ekf_filter_node:
  ros__parameters:
    use_sim_time: true
    frequency: 50.0                    # Filter update rate (Hz)
    sensor_timeout: 0.1                # Sensor timeout (seconds)
    publish_acceleration: false
    publish_tf: false                  # CRITICAL: Set false during SLAM
    
    # Reference frames
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom                  # EKF works in odom frame
```

**Sensor 0 - Wheel Odometry:**
```yaml
    odom0: /diff_cont/odom
    odom0_config: [true,  true,  true,     # x, y, z position - not used
                   false, false, false,    # roll, pitch, yaw - not used
                   true,  true,  false,    # x_vel, y_vel, z_vel - USE
                   false, false, true,     # roll_vel, pitch_vel, yaw_vel - USE yaw_vel
                   false, false, false]    # accelerations - not used
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10
```

**Sensor 1 - IMU:**
```yaml
    imu0: /imu/data
    imu0_config: [false, false, false,     # position - not provided
                  true,  true,  true,      # roll, pitch, yaw - USE
                  false, false, false,     # velocities - not provided
                  true,  true,  true,      # angular velocities - USE
                  true,  true,  true]      # linear accelerations - USE
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true  # CRITICAL
```

**Process Noise Covariance:** (15×15 diagonal matrix)
- Position: [0.05, 0.05, 0.06]
- Orientation: [0.03, 0.03, 0.06]
- Linear velocity: [0.025, 0.025, 0.04]
- Angular velocity: [0.01, 0.01, 0.02]
- Linear acceleration: [0.01, 0.01, 0.015]

**Initial Estimate Covariance:** All diagonal elements = 1e-9

**Output Topic:** `/odometry/local` (remapped from `/odometry/filtered`)

---

### 4. SLAM CONFIGURATION

#### 4.1 SLAM Toolbox (`config/slam_params.yaml`)

**Mode:** mapping

**Core Settings:**
```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    
    # Frame configuration
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    
    # Map settings
    use_map_saver: true
    mode: mapping
    resolution: 0.05                    # 5cm per pixel
    max_laser_range: 12.0
    minimum_time_interval: 0.5
```

**Critical Parameters for Stability:**
```yaml
    minimum_travel_distance: 0.3        # Ignore small movements
    minimum_travel_heading: 0.8         # Ignore rotations <45° (0.8 rad)
    scan_buffer_size: 30                # Larger history buffer
    link_match_minimum_response_fine: 0.8    # Stricter matching
    link_scan_maximum_distance: 0.5     # Close correspondences only
    
    # Loop closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 15   # Reliable loop closure
    loop_match_maximum_variance_coarse: 2.0
    loop_match_minimum_response_coarse: 0.75
    loop_match_minimum_response_fine: 0.8
    
    # Scan matching - prevents "teleportation"
    correlation_search_space_dimension: 0.3   # Local search
    angle_variance_penalty: 2.0               # Penalize rotation errors heavily
```

**Published Topics:**
- `/map` - OccupancyGrid
- `/map_metadata` - MapMetaData

---

### 5. NAVIGATION CONFIGURATION

#### 5.1 Nav2 Parameters (`config/nav2_params.yaml`)

**Key Odometry Source:**
```yaml
# Use fused odometry instead of raw wheel odometry
robot_base_frame: base_link
odom_topic: /odometry/local          # FROM EKF, not /diff_cont/odom
```

**Controller Server:**
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
```

**Planner Server:**
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
```

**Behavior Server:**
```yaml
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
```

**Costmap Settings:**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      width: 3
      height: 3
      resolution: 0.05
      
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
```

---

### 6. LAUNCH FILES

#### 6.1 Basic Simulation (`launch/launch_sim.launch.py`)

**Purpose:** Launch Gazebo with robot and EKF

**Nodes Launched:**
1. Gazebo server + client (with custom world)
2. Robot State Publisher
3. Spawn robot entity
4. Controller spawners:
   - diff_cont
   - joint_broad
   - arm_controller
   - gripper_controller
5. EKF node for sensor fusion

**Key Code:**
```python
def generate_launch_description():
    pkg_name = 'my_robot_controller'
    
    # Process URDF
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 
                              'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # World file
    world_file_path = os.path.join(get_package_share_directory(pkg_name), 
                                   'worlds', 'my_world.world')
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-x', '0', '-y', '0', '-z', '0.4'],
        output='screen'
    )
    
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Controllers
    spawn_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )
    
    # ... (other controller spawners)
    
    # EKF Node
    ekf_config = os.path.join(get_package_share_directory(pkg_name), 
                              'config', 'ekf_params.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        spawn_diff_drive,
        spawn_joint_broad,
        spawn_arm,
        spawn_gripper,
        ekf_node
    ])
```

#### 6.2 SLAM Mapping (`launch/slam_mapping.launch.py`)

**Purpose:** Launch simulation + SLAM Toolbox

**Additional Nodes:**
- Async SLAM Toolbox node (mapping mode)
- Uses `/odometry/local` from EKF
- Publishes map->odom transform

**Critical:** EKF must have `publish_tf: false` to avoid conflict with SLAM

#### 6.3 Keyboard Control (`launch/launch_sim_with_keyboard.launch.py`)

**Purpose:** Launch simulation + keyboard controller

**Additional Node:**
```python
keyboard_controller = Node(
    package='my_robot_controller',
    executable='keyboard_controller.py',
    name='keyboard_controller',
    output='screen'
)
```

---

### 7. PYTHON NODES

#### 7.1 Keyboard Controller (`nodes/controllers/keyboard_controller.py`)

**Purpose:** Teleoperate robot via keyboard

**Controls:**
- **W/A/S/D:** Forward/Left/Backward/Right
- **SPACE:** Stop
- **1/2/3:** Speed modes (slow/medium/fast)
- **I/K:** Shoulder up/down
- **J/L:** Elbow in/out
- **U/M:** Gripper rotate
- **O/P:** Open/close gripper
- **0:** Home position

**Publishers:**
- `/diff_cont/cmd_vel_unstamped` (Twist)
- `/arm_controller/joint_trajectory` (JointTrajectory)
- `/gripper_controller/joint_trajectory` (JointTrajectory)

**Key Features:**
- Speed modes: 0.2, 0.5, 1.0 m/s
- Incremental arm control (±0.1 rad per press)
- Non-blocking keyboard input (termios)
- Clean shutdown on Ctrl+C

**Implementation Notes:**
```python
class EnhancedKeyboardController(Node):
    def __init__(self):
        super().__init__('enhanced_keyboard_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # State tracking
        self.current_speed = 'medium'
        self.arm_positions = [0.0, 0.0, 0.0]  # shoulder, elbow, rotate
        self.gripper_position = 0.0
```

#### 7.2 Auto Mapper (`nodes/mappers/auto_mapper.py`)

**Purpose:** Autonomous exploration for mapping

**Strategy:**
1. Move forward until obstacle detected
2. Rotate to find clear path
3. Continue exploration
4. Build complete map autonomously

**Subscriptions:**
- `/scan` - LaserScan for obstacle detection

**Publishers:**
- `/diff_cont/cmd_vel_unstamped` - Movement commands

**Parameters:**
- Safe distance: 0.5m
- Rotation speed: 0.3 rad/s
- Forward speed: 0.2 m/s

#### 7.3 Compare Odometry (`nodes/compare_odometry.py`)

**Purpose:** Compare raw wheel odom vs fused odom

**Subscriptions:**
- `/diff_cont/odom` - Raw wheel odometry
- `/odometry/local` - Fused odometry (EKF)

**Output:** Prints position/orientation differences

---

### 8. GAZEBO WORLD

#### 8.1 World File (`worlds/my_world.world`)

**Requirements:**
- Flat ground plane
- Walls/obstacles for SLAM testing
- Good lighting
- Physics settings optimized for stability

**Recommended Elements:**
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Walls for SLAM -->
    <!-- Add rectangular walls or maze structure -->
    
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

---

### 9. PACKAGE MANIFEST

#### 9.1 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" 
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>1.0.0</version>
  <description>Mobile robot with SLAM, Nav2, and teleoperation capabilities</description>
  <maintainer email="lamiaehadi14@gmail.com">lamiaehadi</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>controller_manager</depend>
  
  <!-- Navigation and SLAM -->
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>navigation2</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  
  <!-- Messages and Actions -->
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>control_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>
  
  <!-- Gazebo -->
  <depend>gazebo_ros</depend>
  <depend>gazebo_ros_pkgs</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### 9.2 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY description launch config worlds models nodes
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS
    nodes/controllers/ps4_controller.py
    nodes/controllers/keyboard_controller.py
    nodes/mappers/auto_mapper.py
    nodes/mappers/auto_mapper_detailed.py
    nodes/mappers/wall_follower_mapper.py
    nodes/navigation/navigate_to_package.py
    nodes/navigation/set_initial_pose.py
    nodes/navigation/backward_to_wall.py
    nodes/compare_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## 🔄 BUILD AND DEPLOYMENT WORKFLOW

### Step 1: Environment Setup

```bash
# Install ROS2 Humble (if not installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install all dependencies at once
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-twist-mux \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions
```

### Step 2: Workspace Creation

```bash
# Create workspace
mkdir -p ~/ros2_mobile_robot/src
cd ~/ros2_mobile_robot

# Create package
cd src
ros2 pkg create my_robot_controller --build-type ament_cmake

# Create directory structure
cd my_robot_controller
mkdir -p config description launch models nodes/{controllers,mappers,navigation} worlds
```

### Step 3: File Creation Order

**Priority 1 - Core Robot:**
1. `package.xml` - Dependencies
2. `CMakeLists.txt` - Build rules
3. `description/robot.urdf.xacro` - Robot model
4. `description/imu.xacro` - IMU sensor
5. `description/lidar.xacro` - LIDAR sensor

**Priority 2 - Configuration:**
6. `config/my_controllers.yaml` - Controllers
7. `config/ekf_params.yaml` - Sensor fusion
8. `worlds/my_world.world` - Simulation environment

**Priority 3 - Launch & Test:**
9. `launch/launch_sim.launch.py` - Basic simulation
10. Build and test robot spawning

**Priority 4 - Control:**
11. `nodes/controllers/keyboard_controller.py`
12. `launch/launch_sim_with_keyboard.launch.py`
13. Test teleoperation

**Priority 5 - SLAM:**
14. `config/slam_params.yaml`
15. `launch/slam_mapping.launch.py`
16. Test mapping

**Priority 6 - Navigation:**
17. `config/nav2_params.yaml`
18. Navigation launch files
19. Test autonomous navigation

**Priority 7 - Advanced Features:**
20. Mapper nodes (auto_mapper.py, etc.)
21. Navigation helpers
22. Documentation

### Step 4: Build Process

```bash
cd ~/ros2_mobile_robot

# Source ROS2
source /opt/ros/humble/setup.bash

# Build with symlink for faster iteration
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Step 5: Incremental Testing

**Test 1: Robot Spawns**
```bash
ros2 launch my_robot_controller launch_sim.launch.py
# Verify: Robot appears in Gazebo, no errors
```

**Test 2: Controllers Work**
```bash
# Check topics
ros2 topic list | grep diff_cont
ros2 topic echo /diff_cont/odom

# Test movement
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

**Test 3: IMU Publishes**
```bash
ros2 topic hz /imu/data
# Should show ~100 Hz
```

**Test 4: EKF Fuses Data**
```bash
ros2 topic hz /odometry/local
# Should show ~50 Hz

ros2 node info /ekf_filter_node
# Verify subscribers and publishers
```

**Test 5: SLAM Creates Map**
```bash
ros2 launch my_robot_controller slam_mapping.launch.py
# Drive around with keyboard, verify map builds
```

**Test 6: Save Map**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_mobile_robot/maps/my_map
```

**Test 7: Navigation**
```bash
# Load saved map and navigate to waypoints
# Verify robot reaches goals successfully
```

---

## 📊 TOPIC ARCHITECTURE

### Published Topics

| Topic | Type | Publisher | Rate | Description |
|-------|------|-----------|------|-------------|
| `/imu/data` | sensor_msgs/Imu | gazebo_ros_imu_sensor | 100Hz | Raw IMU data |
| `/scan` | sensor_msgs/LaserScan | gazebo_ros_ray_sensor | 20Hz | LIDAR scans |
| `/diff_cont/odom` | nav_msgs/Odometry | diff_drive_controller | 50Hz | Raw wheel odom |
| `/odometry/local` | nav_msgs/Odometry | ekf_filter_node | 50Hz | Fused odometry |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster | 30Hz | All joint positions |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox | 0.5Hz | SLAM-generated map |
| `/tf` | tf2_msgs/TFMessage | multiple | varies | Transform tree |

### Subscribed Topics

| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/diff_cont/cmd_vel_unstamped` | geometry_msgs/Twist | diff_drive_controller | Base movement commands |
| `/arm_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | arm_controller | Arm position commands |
| `/gripper_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | gripper_controller | Gripper commands |

### TF Tree

```
map
 └── odom (published by slam_toolbox OR ekf when publish_tf=true)
      └── base_link (published by diff_drive_controller OR ekf)
           ├── imu_link
           ├── laser_frame
           ├── front_left_link
           ├── front_right_link
           ├── rear_left_link
           ├── rear_right_link
           └── arm_1_link
                └── arm_2_link
                     └── gripper_platform_link
                          ├── gripper_left_link
                          └── gripper_right_link
```

**CRITICAL:** Only ONE node should publish `odom -> base_link` at a time:
- During SLAM: `slam_toolbox` publishes `map -> odom`, `diff_drive_controller` publishes `odom -> base_link`, EKF has `publish_tf: false`
- Without SLAM: EKF can publish `odom -> base_link` with `publish_tf: true`

---

## 🧪 TESTING & VALIDATION

### Unit Tests

Create test scripts for each component:

**Test EKF:**
```bash
#!/bin/bash
# test_ekf.sh
ros2 launch my_robot_controller launch_sim.launch.py &
sleep 10
ros2 topic hz /odometry/local --window 50
# Expected: ~50 Hz
```

**Test SLAM:**
```bash
#!/bin/bash
# test_slam.sh
ros2 launch my_robot_controller slam_mapping.launch.py &
sleep 10
ros2 topic hz /map
# Expected: map updates periodically
```

### Integration Tests

**Full Mapping Workflow:**
1. Launch SLAM mapping
2. Drive robot to explore environment
3. Verify map quality in RViz
4. Save map
5. Verify map files created

**Full Navigation Workflow:**
1. Launch with saved map
2. Set initial pose
3. Send navigation goal
4. Verify robot reaches goal
5. Check path planning quality

---

## 📝 DOCUMENTATION REQUIREMENTS

### README.md Structure

```markdown
# ROS2 Mobile Manipulator Robot

## Overview
[Brief project description]

## Features
- Differential drive mobile base
- 2-DOF arm with gripper
- IMU + wheel encoder fusion
- SLAM mapping
- Autonomous navigation

## Quick Start
[Installation and first run]

## Usage
[Common commands and workflows]

## Architecture
[System diagram and component overview]

## Troubleshooting
[Common issues and solutions]
```

### Installation Guide

Must include:
- System requirements
- Step-by-step ROS2 installation
- Dependency installation
- Workspace build instructions
- Verification steps

### IMU Integration Guide

Must explain:
- Why sensor fusion is needed
- How EKF works
- Configuration parameters
- Tuning instructions
- Testing procedures

---

## 🔧 TUNING & OPTIMIZATION

### EKF Tuning

**If robot motion is jerky:**
- Increase `process_noise_covariance` values
- Decrease `frequency` from 50 to 30 Hz

**If orientation drifts:**
- Verify `imu0_remove_gravitational_acceleration: true`
- Increase IMU orientation weight (elements 3-5 in `imu0_config`)
- Check IMU noise parameters in URDF

**If odometry lags:**
- Increase `frequency` to 100 Hz
- Decrease `sensor_timeout`

### SLAM Tuning

**If map has duplicate walls:**
- Increase `minimum_travel_distance`
- Increase `minimum_travel_heading`
- Increase `link_match_minimum_response_fine`

**If loop closure fails:**
- Increase `loop_match_minimum_chain_size`
- Decrease `loop_match_maximum_variance_coarse`

**If robot "teleports":**
- Increase `angle_variance_penalty`
- Decrease `correlation_search_space_dimension`

### Navigation Tuning

**If robot oscillates:**
- Decrease `controller_frequency`
- Increase `min_vel_x` for smoother motion
- Adjust DWB planner parameters

**If robot gets stuck:**
- Increase inflation radius in costmaps
- Decrease `tolerance` in planner
- Enable recovery behaviors

---

## 🚨 COMMON ISSUES & SOLUTIONS

### Issue 1: Robot Falls Through Ground

**Cause:** Insufficient inertia or collision
**Solution:** Verify `<collision>` elements match `<visual>` in URDF

### Issue 2: Controllers Don't Spawn

**Cause:** Wrong joint names or missing gazebo_ros2_control plugin
**Solution:** Check joint names match between URDF and controller config

### Issue 3: No Odometry Published

**Cause:** diff_drive_controller not running
**Solution:** 
```bash
ros2 control list_controllers
# Verify diff_cont is active
```

### Issue 4: EKF Doesn't Fuse Data

**Cause:** Topics don't match configuration
**Solution:**
```bash
ros2 node info /ekf_filter_node
# Verify subscriptions match config
```

### Issue 5: SLAM Map Drifts

**Cause:** Poor odometry or incorrect SLAM parameters
**Solution:** Use EKF-fused odometry, tune SLAM parameters

### Issue 6: TF Transform Errors

**Cause:** Multiple publishers for same transform
**Solution:** Set `publish_tf: false` in EKF during SLAM

---

## 📦 DELIVERABLES CHECKLIST

### Code Files
- [ ] All URDF/Xacro files created
- [ ] All YAML config files created
- [ ] All Python nodes implemented
- [ ] All launch files created
- [ ] Gazebo world file created
- [ ] package.xml complete
- [ ] CMakeLists.txt complete

### Documentation
- [ ] README.md complete
- [ ] Installation guide written
- [ ] Quickstart guide written
- [ ] IMU integration guide written
- [ ] Architecture documentation
- [ ] Troubleshooting guide

### Testing
- [ ] Robot spawns without errors
- [ ] All controllers functional
- [ ] IMU publishes data
- [ ] EKF fuses data correctly
- [ ] SLAM creates maps
- [ ] Navigation works
- [ ] All Python nodes tested

### Quality
- [ ] No build warnings
- [ ] All nodes handle Ctrl+C gracefully
- [ ] Code follows ROS2 conventions
- [ ] Comments explain complex logic
- [ ] Launch files use proper namespacing

---

## 🎓 LEARNING RESOURCES

### ROS2 Fundamentals
- Official ROS2 Humble Documentation
- `robot_localization` package wiki
- `slam_toolbox` documentation
- Nav2 documentation

### Recommended Reading Order
1. ROS2 basics (nodes, topics, services)
2. URDF/Xacro robot description
3. ros2_control framework
4. Sensor fusion theory
5. SLAM algorithms
6. Navigation stack architecture

---

## 🚀 IMPLEMENTATION STRATEGY FOR AI AGENT

### Phase 1: Foundation (30 minutes)
1. Create workspace structure
2. Create package with manifest files
3. Create basic robot URDF (chassis + wheels only)
4. Create minimal launch file
5. **TEST:** Robot spawns in Gazebo

### Phase 2: Sensors (20 minutes)
6. Add IMU Xacro
7. Add LIDAR Xacro
8. Include sensors in main URDF
9. **TEST:** Sensors publish data

### Phase 3: Control (30 minutes)
10. Create controller configuration
11. Add ros2_control to URDF
12. Update launch file with controller spawners
13. **TEST:** Robot moves with cmd_vel

### Phase 4: Sensor Fusion (25 minutes)
14. Create EKF configuration
15. Add EKF node to launch file
16. **TEST:** `/odometry/local` published at 50Hz

### Phase 5: Teleoperation (20 minutes)
17. Create keyboard controller node
18. Create keyboard launch file
19. **TEST:** Robot controlled via keyboard

### Phase 6: SLAM (30 minutes)
20. Create SLAM configuration
21. Create SLAM launch file
22. Create Gazebo world with walls
23. **TEST:** Map builds while driving

### Phase 7: Advanced Features (45 minutes)
24. Add robotic arm to URDF
25. Create arm controller config
26. Create auto-mapper node
27. Create navigation configuration
28. **TEST:** All systems integrated

### Phase 8: Documentation (30 minutes)
29. Write README
30. Write installation guide
31. Write quickstart guide
32. Add inline code comments

**Total Estimated Time: 3.5-4 hours for complete rebuild**

---

## ✅ ACCEPTANCE CRITERIA

The project is complete when:

1. ✅ `ros2 launch my_robot_controller launch_sim.launch.py` spawns robot without errors
2. ✅ Robot responds to keyboard commands smoothly
3. ✅ `/odometry/local` topic publishes at ~50 Hz
4. ✅ SLAM creates accurate map without drift
5. ✅ Map can be saved successfully
6. ✅ Navigation reaches waypoints reliably
7. ✅ All documentation is clear and complete
8. ✅ Code builds without warnings
9. ✅ All Python nodes are executable
10. ✅ TF tree has no errors

---

## 📌 FINAL NOTES FOR AI AGENT

### Critical Success Factors
1. **Follow file creation order** - Don't skip testing steps
2. **Test incrementally** - Build and test after each phase
3. **Match parameters exactly** - Wheel radius, separation, etc. must match URDF
4. **Frame IDs must be consistent** - base_link, odom, map everywhere
5. **One TF publisher** - Avoid conflicts between EKF and SLAM

### Common AI Agent Mistakes to Avoid
- ❌ Creating all files at once without testing
- ❌ Mismatching joint names between URDF and config
- ❌ Wrong remapping of topics
- ❌ Missing `use_sim_time: true` in configs
- ❌ Forgetting to make Python nodes executable
- ❌ Not including sensors in main URDF via xacro:include

### Recommended Tools
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Monitor node graph
rqt_graph

# Visualize everything
rviz2

# Debug nodes
ros2 node info <node_name>

# Check topics
ros2 topic hz <topic_name>
ros2 topic echo <topic_name>
```

---

## 📜 VERSION HISTORY

- **v2.0** (Jan 2026) - Complete rebuild specification with IMU fusion
- **v1.0** (Previous) - Original project

---

## 📧 CONTACT & SUPPORT

- **Maintainer:** lamiaehadi
- **Email:** lamiaehadi14@gmail.com
- **Repository:** https://github.com/Lamiaehadi/ros2_robot_control_project

---

**END OF SPECIFICATION**

Use this document as your complete guide to rebuild the ROS2 mobile robot project. Follow the implementation strategy phase by phase, test after each phase, and refer to the detailed specifications for exact parameters and configurations.

Good luck! 🚀
