# Resource Optimization Guide for 6GB RAM VM

## ✅ Current VM Specs
- **RAM**: 6GB
- **CPU**: 8 cores
- **Storage**: 50GB
- **OS**: Ubuntu 22.04

## 🎯 Optimizations Applied

### 1. **Gazebo - Headless Mode** ✅
```python
'gui': 'false'  # Saves ~500MB RAM
```

### 2. **Nav2 - Reduced Frequencies** ✅
- Planner: 20Hz → **5Hz** (75% CPU reduction)
- Controller: 20Hz → **10Hz** (50% CPU reduction)

### 3. **SLAM - Lighter Processing** ✅
- Scan throttle: Every 1st → **Every 3rd scan** (66% reduction)
- Preconditioner: SCHUR_JACOBI → **JACOBI** (faster, less memory)

## 📊 Expected Resource Usage

| Component | RAM | CPU (%) |
|-----------|-----|---------|
| Gazebo (headless) | ~800MB | 15-20% |
| SLAM Toolbox | ~400MB | 10-15% |
| Nav2 Stack | ~600MB | 15-20% |
| ROS2 Core + Controllers | ~300MB | 5-10% |
| **Total** | **~2.1GB** | **45-65%** |

**Available**: ~4GB RAM, 35-55% CPU headroom

## 🚀 Additional Optimizations

### A. **Reduce Gazebo Physics Rate**
Edit `src/my_robot_controller/worlds/my_world.world`:
```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Reduced from 0.001 -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Reduced from 1000 -->
</physics>
```

### B. **Limit Costmap Sizes** (If needed)
In `nav2_params.yaml`:
```yaml
global_costmap:
  width: 30  # Reduce from 50
  height: 30
  
local_costmap:
  width: 5  # Reduce from 10
  height: 5
```

### C. **Disable Unused ROS2 Logging**
```bash
export RCUTILS_LOGGING_SEVERITY=INFO  # Change to WARN or ERROR
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

### D. **Use Swap File (Emergency)**
If you hit RAM limits:
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 🔧 Performance Monitoring

### Check Current Usage
```bash
# RAM usage
free -h

# CPU per process
htop

# ROS2 specific
ros2 node list | wc -l  # Count active nodes
ros2 topic hz /scan     # Check sensor frequency
```

### Expected Topic Rates
- `/scan`: ~3-5 Hz (throttled)
- `/odom`: ~50 Hz
- `/map`: ~0.2 Hz (updates every 5s)
- `/tf`: ~20-50 Hz

## ⚡ Quick Performance Tweaks

### Before Launch - Set CPU Governor
```bash
# Performance mode (more responsive, higher power)
sudo cpupower frequency-set -g performance

# Powersave mode (after testing, to save energy)
sudo cpupower frequency-set -g powersave
```

### Reduce Terminal Output
```bash
# Launch with reduced verbosity
ros2 launch my_robot_controller autonomous_mission.launch.py 2>&1 | grep -v "DEBUG"
```

### Clean Build Artifacts
```bash
# Free up storage space
cd /home/lamiae/ROS2_Project/ROS2_Project
rm -rf build/ log/
colcon build --symlink-install --cmake-clean-cache
```

## 🎮 Runtime Monitoring Script

Create `monitor_resources.sh`:
```bash
#!/bin/bash
while true; do
    clear
    echo "========== ROS2 PROJECT RESOURCE MONITOR =========="
    echo ""
    echo "RAM Usage:"
    free -h | grep Mem
    echo ""
    echo "CPU Load (1min, 5min, 15min):"
    uptime | awk -F'load average:' '{print $2}'
    echo ""
    echo "Top ROS2 Processes:"
    ps aux | grep -E "gazebo|slam|nav2|controller" | grep -v grep | awk '{printf "%-20s %5s %5s\n", $11, $3, $4}'
    echo ""
    echo "Disk Usage:"
    df -h /home/lamiae/ROS2_Project
    sleep 5
done
```

Run: `chmod +x monitor_resources.sh && ./monitor_resources.sh`

## 🐛 Troubleshooting Resource Issues

### Symptom: System Freezing
**Cause**: Out of memory
**Solution**: Enable swap file (see section D above)

### Symptom: Navigation sluggish
**Cause**: CPU overload
**Solution**: Reduce `controller_frequency` to 5Hz in `nav2_params.yaml`

### Symptom: SLAM map quality poor
**Cause**: Too much scan throttling
**Solution**: Change `throttle_scans` from 3 to 2

### Symptom: Gazebo crashes
**Cause**: Physics complexity
**Solution**: 
1. Remove unused objects from world file
2. Reduce physics update rate
3. Ensure headless mode enabled

## 📈 Scaling Strategies

### If You Need More Performance:
1. **Increase VM RAM** to 8GB (ideal)
2. **Use pre-built map** instead of SLAM (saves 400MB + CPU)
3. **Reduce world complexity** (fewer objects in Gazebo)
4. **Profile and optimize** specific bottlenecks

### If Resources Still Tight:
1. Run **SLAM separately** first, save map, then use localization mode
2. **Skip Nav2** and use simple odometry navigation
3. **Use Ignition Gazebo** instead of classic Gazebo (lighter)

## ✅ Verification Checklist

Before running mission:
- [ ] Gazebo headless mode enabled
- [ ] SLAM throttle_scans = 3
- [ ] Nav2 planner frequency = 5Hz
- [ ] No unnecessary terminals/apps running
- [ ] Free RAM > 3GB (`free -h`)
- [ ] Swap enabled (optional)
- [ ] CPU governor set to performance

## 🎯 Recommended Testing Sequence

1. **Start with monitoring**:
   ```bash
   htop &
   ```

2. **Launch system**:
   ```bash
   ros2 launch my_robot_controller autonomous_mission.launch.py
   ```

3. **Check stability** (wait 30s, observe RAM/CPU)

4. **Run mission**:
   ```bash
   ros2 run my_robot_controller mission_orchestrator.py
   ```

5. **If issues arise**: Reduce frequencies further in configs

---

**Summary**: Your 6GB RAM VM is sufficient with these optimizations. Expect smooth operation with ~2-3GB RAM usage and 50-60% CPU during autonomous missions.

**Last Updated**: December 19, 2025
