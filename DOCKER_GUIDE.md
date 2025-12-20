# 🐳 Docker Usage Guide

Complete guide for running the ROS2 Mobile Manipulator with Docker.

## Prerequisites

### Install Docker Engine (Recommended)

> ⚠️ **Important**: Use Docker Engine, NOT Docker Desktop, for better GUI and device support on Linux.

```bash
# Install Docker Engine
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Log out and back in, then verify
docker --version
```

### X11 Setup for GUI

```bash
# Allow Docker to access X11 display
xhost +local:docker
```

---

## Quick Start

```bash
# 1. Clone and enter the project
cd ~/projects/ros/Hybrid_robot_control

# 2. Allow X11 access
xhost +local:docker

# 3. Build the Docker images
docker compose build

# 4. Run the simulation
docker compose up sim
```

---

## Available Services

| Service | Command | Description |
|---------|---------|-------------|
| `sim` | `docker compose up sim` | Basic simulation with Gazebo GUI |
| `ros2` | `docker compose up ros2` | Full autonomous mission (SLAM + Nav2) |
| `keyboard` | `docker compose up keyboard` | Simulation with keyboard teleop |
| `ps4` | `docker compose up ps4` | Simulation with PS4 controller |
| `slam` | `docker compose up slam` | SLAM mapping mode |
| `shell` | `docker compose up shell` | Interactive shell for debugging |

---

## Keyboard Control

### Option 1: Standard ROS2 Teleop (Movement Only)

With `sim` running, open a **new terminal**:

```bash
docker compose exec sim bash -c "source /ros2_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped"
```

**Controls:**

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn Left |
| `l` | Turn Right |
| `k` | Stop |
| `u/o` | Diagonal |
| `q/z` | Speed ±10% |

### Option 2: Custom Controller (Base + Arm + Gripper)

```bash
docker compose exec sim bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/install/my_robot_controller/lib/my_robot_controller/keyboard_controller.py"
```

**Controls:**

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `SPACE` | Stop |
| `1` | Arm Extended |
| `2` | Arm Folded |
| `3` | Close Gripper |
| `4` | Open Gripper |
| `E/R` | Rotate Gripper |

> 💡 **Tip**: Click on the terminal window so it captures keyboard input!

---

## Direct Velocity Commands

```bash
# Move forward
docker compose exec sim bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/Twist '{linear: {x: 0.5}}' --once"

# Turn left
docker compose exec sim bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/Twist '{angular: {z: 0.5}}' --once"
```

---

## Arm Control

```bash
# Move arm to extended position
docker compose exec sim bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/JointTrajectory '{
  joint_names: [shoulder_joint, elbow_joint, gripper_rotate_joint],
  points: [{positions: [0.5, -0.5, 0.0], time_from_start: {sec: 1}}]
}' --once"

# Open gripper
docker compose exec sim bash -c "source /ros2_ws/install/setup.bash && \
ros2 topic pub /gripper_controller/joint_trajectory trajectory_msgs/JointTrajectory '{
  joint_names: [gripper_left_joint, gripper_right_joint],
  points: [{positions: [0.03, 0.03], time_from_start: {sec: 1}}]
}' --once"
```

---

## Troubleshooting

### GUI Not Appearing

```bash
# Reset X11 permissions
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY  # Should be :0 or :1
```

### Gazebo Crash (Exit Code 255)

```bash
# Stop all containers and restart
docker compose down
docker compose up sim
```

### Port Already in Use

```bash
# Kill stale processes
docker compose down
docker ps -a | grep ros2 | awk '{print $1}' | xargs -r docker rm -f
```

---

## Stopping the Simulation

```bash
# Press Ctrl+C in the terminal, then:
docker compose down
```

---

## PS4 Controller Setup

1. Connect PS4 controller via Bluetooth or USB
2. Check it's detected: `ls /dev/input/js*`
3. Run: `docker compose up ps4`

**PS4 Controls:**

- Left stick: Drive
- Right stick: Turn
- D-pad: Arm control
- L1/R1: Gripper
