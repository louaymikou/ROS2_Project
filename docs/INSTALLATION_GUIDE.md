# 🚀 Complete Installation Guide - ROS2 Mobile Manipulator Project

## 📌 Table of Contents
1. [System Requirements](#system-requirements)
2. [Install Ubuntu (if needed)](#install-ubuntu)
3. [Install ROS2 Humble](#install-ros2-humble)
4. [Install Project Dependencies](#install-project-dependencies)
5. [Clone and Build Project](#clone-and-build-project)
6. [Verify Installation](#verify-installation)
7. [Run Your First Mission](#run-your-first-mission)
8. [Troubleshooting](#troubleshooting)

---

## 🖥️ System Requirements

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **RAM**: Minimum 4GB (8GB recommended)
- **Storage**: 20GB free space
- **Processor**: 64-bit CPU (x86_64 or ARM64)

---

## 1️⃣ Install Ubuntu (if needed)

### Option A: Dual Boot or Clean Install
1. Download Ubuntu 22.04 LTS ISO: https://ubuntu.com/download/desktop
2. Create bootable USB with Rufus (Windows) or Etcher
3. Boot from USB and follow installation wizard

### Option B: Virtual Machine
1. Download VirtualBox: https://www.virtualbox.org/
2. Create new VM:
   - Type: Linux
   - Version: Ubuntu (64-bit)
   - RAM: 8GB
   - Storage: 50GB dynamic VHD
3. Mount Ubuntu 22.04 ISO and install

### Option C: WSL2 (Windows Subsystem for Linux)
```powershell
# In PowerShell (Admin)
wsl --install -d Ubuntu-22.04
```

**After Ubuntu is installed, open a terminal and update system:**
```bash
sudo apt update
sudo apt upgrade -y
```

**✅ Expected Output:**
```
Hit:1 http://archive.ubuntu.com/ubuntu jammy InRelease
Get:2 http://archive.ubuntu.com/ubuntu jammy-updates InRelease [119 kB]
...
Reading package lists... Done
Building dependency tree... Done
...
Done
```

---

## 2️⃣ Install ROS2 Humble

### Step 1: Set Locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**✅ Expected Output:**
```
Generating locales (this might take a while)...
  en_US.UTF-8... done
Generation complete.
```

### Step 2: Enable Ubuntu Universe Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

**✅ Expected Output:**
```
'universe' distribution component is already enabled for all sources.
```

### Step 3: Add ROS2 GPG Key
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

**✅ Expected Output:**
```
curl is already the newest version (7.81.0-1ubuntu1.x)
```
(No output from curl command means success)

### Step 4: Add ROS2 Repository
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 5: Install ROS2 Humble Desktop
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
```

**⏱️ This will take 10-20 minutes depending on internet speed**

**✅ Expected Output:**
```
Reading package lists... Done
Building dependency tree... Done
The following NEW packages will be installed:
  ros-humble-desktop ros-humble-ros-base ros-humble-rviz2 ...
...
Setting up ros-humble-desktop (0.10.0-1jammy.20230612.195137) ...
Processing triggers for libc-bin (2.35-0ubuntu3) ...
```

### Step 6: Install Development Tools
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

**✅ Expected Output:**
```
Reading package lists... Done
The following NEW packages will be installed:
  python3-colcon-common-extensions python3-rosdep
...
Setting up python3-colcon-common-extensions (0.3.0-1) ...
```

### Step 7: Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

**✅ Expected Output:**
```
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run
  rosdep update

reading in sources list data from /etc/ros/rosdep/sources.list.d
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
...
updated cache in /home/user/.ros/rosdep/sources.cache
```

### Step 8: Setup ROS2 Environment (Add to ~/.bashrc)
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 9: Verify ROS2 Installation
```bash
ros2 --version
```

**✅ Expected Output:**
```
ros2 cli version 0.18.9
```

**✅ Test a simple ROS2 command:**
```bash
ros2 topic list
```

**✅ Expected Output:**
```
/parameter_events
/rosout
```

---

## 3️⃣ Install Project Dependencies

### Install Gazebo Simulation
```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

**✅ Expected Output:**
```
Reading package lists... Done
The following NEW packages will be installed:
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros ...
...
Setting up ros-humble-gazebo-ros-pkgs (3.7.0-2jammy.20230612.225604) ...
```

### Install Navigation & SLAM
```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

**✅ Expected Output:**
```
Reading package lists... Done
The following NEW packages will be installed:
  ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-slam-toolbox ...
...
Processing triggers for libc-bin (2.35-0ubuntu3) ...
```

### Install ros2_control
```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control
```

### Install Teleop Tools (Optional)
```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

### Install Additional Python Dependencies
```bash
sudo apt install -y \
  python3-pip \
  python3-transforms3d \
  python3-numpy
```

---

## 4️⃣ Clone and Build Project

### Step 1: Create Workspace Directory
```bash
mkdir -p ~/ROS2_Project/ROS2_Project
cd ~/ROS2_Project/ROS2_Project
pwd
```

**✅ Expected Output:**
```
/home/your_username/ROS2_Project/ROS2_Project
```

### Step 2: Clone the Project
```bash
git clone https://github.com/Lamiaehadi/Hybrid_robot_control.git .
```

**✅ Expected Output:**
```
Cloning into '.'...
remote: Enumerating objects: 1030, done.
remote: Counting objects: 100% (1030/1030), done.
remote: Compressing objects: 100% (361/361), done.
remote: Total 1030 (delta 550), reused 1030 (delta 550), pack-reused 0
Receiving objects: 100% (1030/1030), 398.71 KiB | 113.00 KiB/s, done.
Resolving deltas: 100% (550/550), done.
```

**Or if you already have the files, copy them to `~/ROS2_Project/ROS2_Project`**

### Step 3: Verify Project Structure
```bash
ls
```

**✅ Expected Output:**
```
AUTONOMOUS_SYSTEM_PLAN.md  INSTALLATION_GUIDE.md  README.md  src
IMPLEMENTATION_COMPLETE.md QUICK_REFERENCE.md     RESOURCE_OPTIMIZATION.md
index.html                 SETUP_AND_USAGE_GUIDE.md QUICK_SUMMARY.md
```

**✅ Verify src directory:**
```bash
ls src/
```

**✅ Expected Output:**
```
my_robot_controller
```

### Step 4: Install Dependencies with rosdep
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

**✅ Expected Output:**
```
#All required rosdeps installed successfully
```

Or if dependencies are already installed:
```
#All required rosdeps installed successfully
Executing command ['sudo', 'apt-get', 'install', '-y', 'ros-humble-xacro']
...
```

### Step 5: Build the Workspace
```bash
colcon build --symlink-install
```

**⏱️ First build takes 2-5 minutes**

**✅ Expected Output:**
```
Starting >>> my_robot_controller
Finished <<< my_robot_controller [2.45s]

Summary: 1 package finished [2.67s]
```

**🎉 If you see this, your build was successful!**

### Step 6: Source the Workspace
```bash
source install/setup.bash
```

### Step 7: Add to ~/.bashrc (Optional - Auto-source on terminal startup)
```bash
echo "source ~/ROS2_Project/ROS2_Project/install/setup.bash" >> ~/.bashrc
```

---

## 5️⃣ Verify Installation

### Check ROS2 Packages
```bash
ros2 pkg list | grep my_robot_controller
```

**✅ Expected Output:**
```
my_robot_controller
```

### Check Launch Files
```bash
ros2 launch my_robot_controller --show-args autonomous_mission.launch.py
```

**✅ Expected Output:**
```
Arguments (pass arguments as '<name>:=<value>'):

    'use_sim_time':
        Use simulation (Gazebo) clock if true
        (default: 'true')

    'world':
        Gazebo world file
        (default: FindPackageShare(pkg='my_robot_controller') + '/worlds/my_world.world')
...
```

### Test Gazebo
```bash
gazebo --version
```

**✅ Expected Output:**
```
Gazebo multi-robot simulator, version 11.10.2
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org
```

---

## 6️⃣ Run Your First Mission

### Terminal 1: Launch System
```bash
cd ~/ROS2_Project/ROS2_Project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_controller autonomous_mission.launch.py
```

**⏳ Wait 15-20 seconds for Nav2 to initialize**

**✅ Expected Output (partial - lots of messages):**
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gzserver-1]: process started with pid [12345]
[INFO] [gzclient-2]: process started with pid [12346]
[INFO] [robot_state_publisher-3]: process started with pid [12347]
...
[INFO] [slam_toolbox-x]: Starting SLAM Toolbox node
[INFO] [lifecycle_manager-x]: Creating lifecycle manager
[INFO] [bt_navigator-x]: Configuring
[INFO] [controller_server-x]: Configuring
...
[INFO] [lifecycle_manager-x]: Activating...
[INFO] [lifecycle_manager-x]: All nodes activated
```

**✅ Visual Confirmation:**
- Gazebo window opens showing robot and environment
- RViz2 opens showing robot model and sensor data
- Green laser scan lines visible in RViz2
- Map building in RViz2 (starts black, fills in as SLAM runs)

### Terminal 2: Start Mission (Open new terminal)
```bash
cd ~/ROS2_Project/ROS2_Project
source install/setup.bash
ros2 run my_robot_controller mission_orchestrator.py
```

**✅ Expected Output:**
```
[INFO] Mission Orchestrator started!
[INFO] Waiting for action servers...
[INFO] Action servers ready!
[INFO] Starting autonomous mission: A -> B -> A

=== PHASE 1: Navigate to Pickup Point A ===
[INFO] Sending goal to navigate to point A (0.0, 5.5)
[INFO] Navigation in progress...
[INFO] Goal reached! Arrived at point A

=== PHASE 2: Pick Object ===
[INFO] Executing pick action...
[INFO] Moving arm to pre-grasp position
[INFO] Lowering arm to object
[INFO] Closing gripper
[INFO] Lifting object
[INFO] Pick complete!

=== PHASE 3: Navigate to Dropoff Point B ===
[INFO] Sending goal to navigate to point B (3.0, 0.0)
[INFO] Navigation in progress...
[INFO] Goal reached! Arrived at point B

=== PHASE 4: Place Object ===
[INFO] Executing place action...
[INFO] Lowering arm
[INFO] Opening gripper
[INFO] Releasing object
[INFO] Retracting arm
[INFO] Place complete!

=== PHASE 5: Return to Point A ===
[INFO] Sending goal to return to point A (0.0, 5.5)
[INFO] Navigation in progress...
[INFO] Goal reached! Back at point A

[INFO] ✅ MISSION COMPLETE! ✅
[INFO] Full cycle completed successfully
```

**🎉 Watch the robot autonomously:**
1. Navigate to cube location (watch in Gazebo & RViz2)
2. Pick cube with arm + gripper (gripper closes on cube)
3. Navigate to drop-off point (robot moves with cube)
4. Place cube (gripper opens, cube drops)
5. Return to start position

**⏱️ Mission duration: ~2-3 minutes**

---

## 7️⃣ Alternative Usage Modes

### Keyboard Control (All-in-One)
```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py
```

**Controls:**
- `W` - Forward
- `S` - Backward
- `A` - Turn Left
- `D` - Turn Right
- `SPACE` - Stop
- `Q/E` - Arm Joint 1
- `R/F` - Arm Joint 2
- `T/G` - Arm Joint 3
- `Y/H` - Gripper Open/Close

### Manual Control with Separate Terminals

**Terminal 1 - Simulation:**
```bash
source install/setup.bash
ros2 launch my_robot_controller launch_sim.launch.py
```

**Terminal 2 - Keyboard Control:**
```bash
source install/setup.bash
python3 src/my_robot_controller/keyboard_controller.py
```

---

## 8️⃣ Troubleshooting

### Issue: "Package not found" error
**Solution:**
```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_Project/ROS2_Project/install/setup.bash
```

### Issue: "colcon: command not found"
**Solution:**
```bash
sudo apt install python3-colcon-common-extensions
```

### Issue: Nav2 won't start or timeout
**Solution:**
- Wait longer (up to 30 seconds on slower machines)
- Check if SLAM is running: `ros2 topic list | grep map`
- Verify system resources (4GB RAM minimum)

### Issue: Robot doesn't move
**Solution:**
```bash
# Verify controllers
ros2 control list_controllers
```

**✅ Expected Output:**
```
diff_cont[diff_drive_controller/DiffDriveController] active
arm_controller[position_controllers/JointTrajectoryController] active
gripper_controller[position_controllers/JointTrajectoryController] active
joint_broad[joint_state_broadcaster/JointStateBroadcaster] active
```

All controllers should show **active** status. If not, wait a few more seconds or restart the launch.

### Issue: Gazebo crashes or high CPU usage
**Solution:**
- See `RESOURCE_OPTIMIZATION.md` for performance tuning
- Run headless mode (no GUI): Add `headless:=true` to launch commands
- Close unnecessary applications

### Issue: "No module named 'transforms3d'"
**Solution:**
```bash
pip3 install transforms3d
```

### Issue: Build fails with errors
**Solution:**
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Issue: Git clone fails
**Solution:**
```bash
# Configure git
git config --global user.name "Your Name"
git config --global user.email "your@email.com"

# Try SSH instead of HTTPS (if you have SSH keys)
git clone git@github.com:Lamiaehadi/Hybrid_robot_control.git .
```

---

## 🆘 Getting Help

### Check System Status
```bash
# ROS2 installation
ros2 doctor

# Topics being published
ros2 topic list

# Nodes running
ros2 node list

# Controller status
ros2 control list_controllers
```

### Useful Commands
```bash
# Kill all ROS2 processes
killall -9 ros2

# Kill Gazebo
killall -9 gzserver gzclient

# View logs
ros2 run my_robot_controller mission_orchestrator.py 2>&1 | tee mission.log
```

---

## 📚 Additional Resources

- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **Gazebo Tutorials**: https://gazebosim.org/docs
- **Nav2 Documentation**: https://navigation.ros.org/
- **Project Documentation**:
  - `README.md` - Quick reference
  - `AUTONOMOUS_SYSTEM_PLAN.md` - Technical architecture
  - `SETUP_AND_USAGE_GUIDE.md` - Detailed usage
  - `RESOURCE_OPTIMIZATION.md` - Performance tuning

---

## ✅ Installation Checklist

- [ ] Ubuntu 22.04 installed and updated
- [ ] ROS2 Humble installed (`ros2 --version` works)
- [ ] rosdep initialized
- [ ] Project dependencies installed
- [ ] Workspace cloned to `~/ROS2_Project/ROS2_Project`
- [ ] `colcon build` completed successfully
- [ ] Workspace sourced (`source install/setup.bash`)
- [ ] Launch files detected (`ros2 launch my_robot_controller --show-args autonomous_mission.launch.py`)
- [ ] First mission runs successfully

---

**🎯 Estimated Total Setup Time:** 1-2 hours (depending on internet speed and system performance)

**Last Updated:** December 20, 2025  
**Compatible with:** ROS2 Humble on Ubuntu 22.04 LTS
