# 🎯 What is an IMU (Inertial Measurement Unit)?

An **IMU** is a sensor that measures a robot's **motion and orientation** by detecting forces and rotation. It's like giving your robot a sense of balance and motion awareness - similar to your inner ear for humans.

---

## 📊 What an IMU Measures

### 1. Angular Velocity (Gyroscope)
- **What:** How fast the robot is rotating
- **Units:** rad/s (radians per second)
- **Axes:** Roll, Pitch, Yaw (X, Y, Z rotation)
- **Example:** When your robot turns left, the gyroscope detects rotation around the Z-axis

### 2. Linear Acceleration (Accelerometer)
- **What:** How fast the robot is speeding up or slowing down
- **Units:** m/s² (meters per second squared)
- **Axes:** X, Y, Z directions
- **Example:** When robot accelerates forward, accelerometer detects positive X acceleration

### 3. Orientation (Computed)
- **What:** Robot's tilt and heading angle
- **Units:** Quaternion or Euler angles (roll, pitch, yaw)
- **How:** Computed by integrating gyroscope data over time

---

## 🔧 IMU Configuration in Your Robot

### Physical Specifications
```yaml
Location: Mounted at (0.3, 0, 0.15) on robot chassis
Size: 5cm × 5cm × 2cm (small green box)
Mass: 0.015 kg
Update Rate: 100 Hz (100 measurements per second)
```

### ROS2 Configuration
```yaml
Topic: /imu/data
Message Type: sensor_msgs/Imu
Frame: imu_link
Always On: true
Visualize: true (visible in Gazebo/RViz)
```

### IMU Data Structure
```yaml
orientation:         # Which way robot is facing (quaternion)
  x, y, z, w
angular_velocity:    # How fast robot is spinning
  x, y, z            # (roll_rate, pitch_rate, yaw_rate)
linear_acceleration: # How fast robot is accelerating
  x, y, z            # (forward/back, left/right, up/down)
```

### Noise Characteristics
Your IMU includes realistic noise to simulate real-world sensors:

```yaml
Angular Velocity Noise:
  - Standard Deviation: 2e-4 rad/s
  - Type: Gaussian (random)
  
Linear Acceleration Noise:
  - Standard Deviation: 1.7e-2 m/s²
  - Type: Gaussian (random)
```

---

## 🤖 Why You Need IMU in Your Project

### Problem Without IMU ❌

**Wheel encoders alone are unreliable:**
- Wheels slip on smooth floors → wrong position estimate
- Wheel diameter changes (wear, tire pressure)
- Can't detect if robot is being pushed/dragged
- Rotation estimates drift over time
- Accumulates errors during turns
- No independent verification of movement

### Solution With IMU ✅

**IMU provides independent measurements:**
- ✅ Detects actual rotation (not just wheel rotation)
- ✅ Knows if robot is slipping or sliding
- ✅ Provides accurate orientation
- ✅ Updates at 100 Hz (very responsive)
- ✅ Doesn't depend on wheel contact
- ✅ Works even if wheels are off the ground

---

## 🔄 How IMU Works in Your Robot

### Sensor Fusion Architecture

```
┌─────────────────┐              ┌─────────────────┐
│ Wheel Encoders  │              │   IMU Sensor    │
│  (4 wheels)     │              │ (gyro + accel)  │
└────────┬────────┘              └────────┬────────┘
         │                                │
         ↓ 50 Hz                          ↓ 100 Hz
┌─────────────────┐              ┌─────────────────┐
│ /diff_cont/odom │              │   /imu/data     │
│ - Position      │              │ - Orientation   │
│ - Velocity      │              │ - Angular vel   │
└────────┬────────┘              │ - Acceleration  │
         │                       └────────┬────────┘
         │                                │
         └───────────┬────────────────────┘
                     ↓
         ┌───────────────────────┐
         │   EKF Filter Node     │
         │ (robot_localization)  │
         │   Fuses both sources  │
         └───────────┬───────────┘
                     ↓
         ┌───────────────────────┐
         │   /odometry/local     │
         │  (Fused, accurate)    │
         └───────────────────────┘
```

### What Each Sensor Contributes

| Data Type | Wheel Encoders | IMU | EKF Decision |
|-----------|----------------|-----|--------------|
| **X/Y Position** | ✅ Primary source | ❌ Not measured | Uses encoders |
| **X/Y Velocity** | ✅ Computed from wheels | ❌ Not directly | Uses encoders |
| **Orientation (Yaw)** | ⚠️ Drifts over time | ✅ Very accurate | **IMU corrects encoders** |
| **Angular Velocity** | ⚠️ Indirect estimate | ✅ Direct measurement | **Prefers IMU** |
| **Linear Acceleration** | ❌ Not measured | ✅ Direct measurement | **Uses IMU** |
| **Slip Detection** | ❌ Can't detect | ✅ Detects mismatch | **IMU alerts to slippage** |

---

## 📱 Real-World Example: Your Smartphone

Your smartphone has the same sensors as your robot's IMU:

### Accelerometer
- Detects when you tilt your phone
- Auto-rotates screen (portrait/landscape)
- Counts steps when you walk
- Detects shaking for "undo"

### Gyroscope
- Detects rotation for games
- Stabilizes photos/videos
- Enables 360° panorama photos
- VR/AR head tracking

### Your Robot's IMU
**Does the same thing** - gives the robot awareness of:
- Which way it's facing
- How fast it's turning
- If it's tilting or level
- If it's accelerating or braking

---

## 🎮 Practical Benefits in Your Project

### 1. Better Mapping (SLAM)

**Without IMU:**
```
Problem: Map has duplicate/ghost walls
Reason: Rotation errors accumulate
Result: Unusable maps, navigation fails
```

**With IMU:**
```
Solution: Clean, accurate maps
Reason: Correct rotation tracking from gyroscope
Result: Reliable navigation, loop closure works
```

### 2. Better Navigation

**Without IMU:**
```
Command: "Turn 90 degrees left"
Result: Robot overshoots to 95 degrees or undershoots to 85 degrees
Effect: Zigzag path, hits walls, misses goals
```

**With IMU:**
```
Command: "Turn 90 degrees left"
Result: Robot turns exactly 90 degrees (±1 degree)
Effect: Smooth turns, accurate heading, reaches goals
```

### 3. Slip Detection

**Real Scenario:**
```
Situation: Robot wheels spin on smooth tile floor

Wheel Encoders Report:
  ✗ "Wheels rotated 10 times"
  ✗ "Therefore, we moved 1 meter forward!"
  
IMU Reports:
  ✓ "No linear acceleration detected"
  ✓ "Position unchanged"
  ✓ "We're slipping!"
  
EKF Decision:
  → Trust the IMU
  → Wheels are slipping
  → Actual movement: 0 meters
  → Increase covariance (uncertainty)
```

### 4. Dynamic Obstacle Avoidance

**Scenario: Someone pushes your robot**
```
Wheel Encoders: "No wheel rotation detected, we're stationary"
IMU:            "Acceleration detected! We're being moved!"
Result:         Robot knows it's being pushed, can react appropriately
```

---

## 🔍 How to View Your IMU Data

### Command Line

```bash
# See raw IMU measurements (real-time)
ros2 topic echo /imu/data

# Check update rate (should be ~100 Hz)
ros2 topic hz /imu/data

# See IMU data in human-readable format
ros2 topic echo /imu/data --once

# Monitor IMU alongside wheel odometry
ros2 topic echo /diff_cont/odom /imu/data
```

### RViz Visualization

```bash
# Launch RViz
rviz2

# Add IMU visualization:
# 1. Click "Add" button
# 2. Select "By topic" tab
# 3. Find "/imu/data"
# 4. Choose "Imu" display type
# 5. Click OK

# You'll see:
# - Arrows showing acceleration direction
# - Orientation visualization
# - Real-time updates at 100 Hz
```

### Expected Output Example

```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: imu_link

orientation:
  x: 0.0      # No roll
  y: 0.0      # No pitch
  z: 0.707    # Robot facing at 90 degrees (pointing left)
  w: 0.707    # Quaternion W component

angular_velocity:
  x: 0.0      # Not rolling (tilting sideways)
  y: 0.0      # Not pitching (tilting forward/back)
  z: 0.5      # Turning left at 0.5 rad/s (~28.6 deg/s)

linear_acceleration:
  x: 2.0      # Accelerating forward at 2 m/s²
  y: 0.0      # No sideways acceleration
  z: 9.81     # Gravity pulling down (always present)
```

---

## 🧮 Understanding IMU Coordinates

### Reference Frame
```
IMU mounted on robot chassis at (0.3, 0, 0.15)

Robot Coordinate System:
  X-axis: Forward (red arrow)
  Y-axis: Left (green arrow)
  Z-axis: Up (blue arrow)
  
IMU Measurements:
  angular_velocity.x: Roll rate (rotate around X-axis)
  angular_velocity.y: Pitch rate (rotate around Y-axis)
  angular_velocity.z: Yaw rate (rotate around Z-axis) ← Most important for navigation
  
  linear_acceleration.x: Forward/backward acceleration
  linear_acceleration.y: Left/right acceleration
  linear_acceleration.z: Up/down acceleration (includes gravity)
```

### Orientation as Quaternion

Your IMU reports orientation as a **quaternion** (x, y, z, w):

```python
# Convert quaternion to Euler angles (degrees)
import math

def quaternion_to_euler(x, y, z, w):
    # Roll (rotation around X-axis)
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    
    # Pitch (rotation around Y-axis)
    pitch = math.asin(2*(w*y - z*x))
    
    # Yaw (rotation around Z-axis) - heading angle
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    
    # Convert to degrees
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# Example:
roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, 0.707, 0.707)
print(f"Heading: {yaw}°")  # Output: Heading: 90.0° (pointing left)
```

---

## ⚙️ EKF Configuration for IMU

Your robot uses these IMU measurements in the EKF filter:

```yaml
# From config/ekf_params.yaml

imu0: /imu/data
imu0_config: [
    false, false, false,    # x, y, z position - IMU doesn't measure position
    true,  true,  true,     # roll, pitch, yaw - ✅ USE orientation from IMU
    false, false, false,    # x_vel, y_vel, z_vel - IMU doesn't measure velocity
    true,  true,  true,     # roll_vel, pitch_vel, yaw_vel - ✅ USE angular velocities
    true,  true,  true      # x_acc, y_acc, z_acc - ✅ USE linear accelerations
]

imu0_remove_gravitational_acceleration: true  # CRITICAL: Remove gravity from Z-axis
```

**What this means:**
- ✅ **Orientation:** IMU provides accurate roll/pitch/yaw
- ✅ **Angular velocity:** IMU directly measures rotation rates
- ✅ **Acceleration:** IMU measures how robot speeds up/slows down
- ❌ **Position:** IMU cannot measure absolute position (encoders do this)
- ❌ **Velocity:** IMU cannot directly measure velocity (computed from integration)

---

## 🔬 Technical Details

### Sensor Fusion Mathematics

The EKF (Extended Kalman Filter) fuses IMU and encoder data using this process:

```
1. PREDICTION (uses motion model):
   - Predict new position based on velocity
   - Predict new velocity based on acceleration (from IMU)
   - Add uncertainty (process noise)

2. UPDATE (uses sensor measurements):
   - Get encoder odometry: /diff_cont/odom
   - Get IMU data: /imu/data
   - Calculate Kalman gain (how much to trust each sensor)
   
3. CORRECTION:
   - Weighted average of prediction and measurements
   - High-quality sensors get more weight
   - Output: /odometry/local (optimal estimate)
   
4. REPEAT at 50 Hz
```

### Why 100 Hz Update Rate?

Your IMU updates at **100 Hz** (100 times per second) because:

✅ **Fast rotation detection** - Catches quick turns  
✅ **Smooth data** - No gaps or jumps  
✅ **Low latency** - Only 10ms between updates  
✅ **Better integration** - More accurate orientation tracking  
✅ **Matches human perception** - Similar to our inner ear  

The EKF runs at 50 Hz, using the latest IMU data available.

---

## 📚 Simple Analogy

### IMU is Like Your Inner Ear

Think about how you know you're moving:

| Human Sense | Robot Sensor | Measures |
|-------------|--------------|----------|
| **Eyes** 👀 | LIDAR/Camera | See the world, obstacles |
| **Inner Ear** 👂 | **IMU** | Feel balance, rotation, acceleration |
| **Feet/Legs** 🦵 | Wheel Encoders | Know how much you walked |
| **Brain** 🧠 | EKF Filter | Combine all senses for awareness |

**Example:**
- When you close your eyes and spin around, you still know you're spinning (inner ear = IMU)
- When you're in a car, you feel acceleration even with eyes closed (inner ear = IMU accelerometer)
- When standing on a tilting surface, you know you're tilted (inner ear = IMU orientation)

Your robot's IMU gives it this same awareness! 🎯

---

## 🧪 Testing Your IMU

### Quick Test Script

Save this as `test_imu.sh`:

```bash
#!/bin/bash
echo "🔍 Testing IMU Sensor..."
echo ""

echo "1. Checking if IMU topic exists..."
ros2 topic list | grep imu/data
if [ $? -eq 0 ]; then
    echo "✅ IMU topic found"
else
    echo "❌ IMU topic not found. Is simulation running?"
    exit 1
fi

echo ""
echo "2. Checking IMU update rate..."
timeout 5 ros2 topic hz /imu/data
echo ""

echo "3. Checking IMU data (5 seconds)..."
timeout 5 ros2 topic echo /imu/data
echo ""

echo "4. Verifying EKF is using IMU..."
ros2 node info /ekf_filter_node | grep imu
echo ""

echo "✅ IMU Test Complete!"
```

Make executable and run:
```bash
chmod +x test_imu.sh
./test_imu.sh
```

### Expected Results

```
✅ IMU topic found
✅ Update rate: ~100 Hz
✅ Data published with orientation, angular_velocity, linear_acceleration
✅ EKF node is subscribed to /imu/data
```

---

## 🎓 Learning Resources

### Understanding IMU Concepts
- **Quaternions:** 4D representation of 3D rotation (avoids gimbal lock)
- **Euler Angles:** Roll/Pitch/Yaw (easier to understand, can have singularities)
- **Sensor Fusion:** Combining multiple sensors for better estimates
- **Kalman Filter:** Optimal estimation algorithm for noisy sensors

### Related ROS2 Packages
- `robot_localization` - EKF sensor fusion (what you're using)
- `imu_filter_madgwick` - Alternative IMU orientation filter
- `imu_complementary_filter` - Simple IMU filter
- `robot_pose_ekf` - Older ROS1 package (deprecated)

### Further Reading
- [sensor_msgs/Imu Message Documentation](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)
- [robot_localization Package Wiki](http://docs.ros.org/en/humble/p/robot_localization/)
- [Understanding IMU Sensors](https://www.vectornav.com/resources/inertial-navigation-primer)

---

## 🔧 Troubleshooting

### IMU Not Publishing

```bash
# Check if IMU topic exists
ros2 topic list | grep imu

# If not found:
# 1. Verify simulation is running
# 2. Check imu.xacro is included in robot.urdf.xacro
# 3. Rebuild workspace: colcon build --symlink-install
```

### Wrong Update Rate

```bash
# Expected: ~100 Hz
ros2 topic hz /imu/data

# If different:
# - Check <update_rate>100</update_rate> in imu.xacro
# - Verify Gazebo is running in real-time
```

### EKF Not Using IMU

```bash
# Check EKF subscriptions
ros2 node info /ekf_filter_node

# Should see:
# Subscribers:
#   /imu/data: sensor_msgs/msg/Imu

# If not:
# 1. Check config/ekf_params.yaml
# 2. Verify imu0: /imu/data
# 3. Verify imu0_config has 'true' values
```

### IMU Data Looks Wrong

```bash
# Check for NaN values
ros2 topic echo /imu/data | grep nan

# Check orientation is normalized (quaternion length = 1)
# x² + y² + z² + w² should equal 1.0

# Verify linear_acceleration.z ≈ 9.81 when stationary (gravity)
```

---

## ✅ Summary

### What You Should Remember

1. **IMU = Motion Sensor** - Measures rotation and acceleration
2. **100 Hz Update** - Fast, responsive measurements
3. **Sensor Fusion** - Combines with wheel encoders for better odometry
4. **Key Benefits:**
   - Accurate orientation tracking
   - Slip detection
   - Better SLAM and navigation
   - Independent motion verification

### Quick Commands

```bash
# View IMU data
ros2 topic echo /imu/data

# Check update rate
ros2 topic hz /imu/data

# See fused odometry (IMU + encoders)
ros2 topic echo /odometry/local

# Compare raw vs fused odometry
python3 src/my_robot_controller/nodes/compare_odometry.py
```

**Your robot is smarter because it has an IMU!** 🚀
