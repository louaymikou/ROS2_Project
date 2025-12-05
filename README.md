si on veux lancer la simulation avec le controlleur :
  colcon build
  source install/setup.bash
  ros2 launch my_robot_controller launch_sim_with_ps4.launch.py


si on veux lancer la simulation sans le controlleur :
1er terminal:
  colcon build
  source install/setup.bash
  ros2 launch my_robot_controller launch_sim.launch.py
2eme terminal :
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
3eme terminal :
  python3 /home/wayay/ROS_PROJECT/src/my_robot_controller/simple_arm_control.py
