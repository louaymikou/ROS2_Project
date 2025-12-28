#!/bin/bash
cd /home/ikram/ROS2_Project
source install/setup.bash

# Lancement SLAM avec remapping explicite
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file src/my_robot_controller/config/slam_params.yaml \
  -p use_sim_time:=true \
  --remap /odom:=/diff_cont/odom
