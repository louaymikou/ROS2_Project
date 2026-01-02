si on veux lancer la simulation avec la manette PS4 (tout-en-un) :
  colcon build
  source install/setup.bash
  ros2 launch my_robot_controller launch_sim_with_ps4.launch.py


si on veux lancer la simulation avec le clavier QWERTY (tout-en-un) :
  colcon build
  source install/setup.bash
  ros2 launch my_robot_controller launch_sim_with_keyboard.launch.py


si on veux lancer la simulation avec le suivi de ligne automatique (OPTIMISÉ) :
  colcon build
  source install/setup.bash
  ros2 launch my_robot_controller launch_sim_with_line_follower.launch.py

Pour visualiser ce que voit la caméra en temps réel :
  Terminal 2: python3 /home/wayay/ROS_PROJECT/src/my_robot_controller/simple_camera_view.py

Pour le debug avancé avec masque binaire et détection :
  Terminal 2: python3 /home/wayay/ROS_PROJECT/src/my_robot_controller/camera_debug.py


si on veux lancer la simulation ET le contrôleur clavier séparément :
1er terminal:
  colcon build
  source install/setup.bash
  ros2 launch my_robot_controller launch_sim.launch.py
2eme terminal :
  source install/setup.bash
  python3 /home/wayay/ROS_PROJECT/src/my_robot_controller/keyboard_controller.py


si on veux lancer la simulation avec contrôle manuel :
1er terminal:
  colcon build
  source install/setup.bash
  ros2 launch my_robot_controller launch_sim.launch.py
2eme terminal :
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
3eme terminal :
  python3 /home/wayay/ROS_PROJECT/src/my_robot_controller/simple_arm_control.py


NOTES SUR LE SUIVI DE LIGNE :
- L'algorithme est basé sur les tutoriels The Construct (OpenCV + ROS2)
- Détection de lignes NOIRES sur fond GRIS/BLANC via masque HSV
- Contrôle proportionnel simple (Kp) pour un suivi stable
- Paramètres ajustables :
    * linear_speed : vitesse constante (défaut: 0.25 m/s)
    * kp : constante proportionnelle (défaut: 0.015)
    * min_area : surface minimale de détection (défaut: 500 pixels)
- Le robot doit être positionné sur ou près de la ligne au démarrage

