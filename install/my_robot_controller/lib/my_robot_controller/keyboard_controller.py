#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

# Texte d'aide
msg = """
╔════════════════════════════════════════════╗
║   CONTRÔLE DU ROBOT AU CLAVIER QWERTY      ║
╠════════════════════════════════════════════╣
║ MOUVEMENT DE LA BASE:                      ║
║   Z ou W : Avancer                         ║
║   S      : Reculer                         ║
║   Q ou A : Pivoter à gauche                ║
║   D      : Pivoter à droite                ║
║   ESPACE : Arrêter                         ║
║                                            ║
║ COMMANDES DU BRAS:                         ║
║   1 : Position étendue                     ║
║   2 : Position rangée                      ║
║                                            ║
║ COMMANDES DE LA PINCE:                     ║
║   3 : Fermer la pince                      ║
║   4 : Ouvrir la pince                      ║
║                                            ║
║ ROTATION DE LA PINCE:                      ║
║   E : Rotation anti-horaire                ║
║   R : Rotation horaire                     ║
║                                            ║
║ QUITTER: Ctrl+C                            ║
╚════════════════════════════════════════════╝
"""

class KeyboardRobotController(Node):
    def __init__(self):
        super().__init__('keyboard_robot_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Mémoriser la position du bras
        self.shoulder_pos = 0.0
        self.elbow_pos = 0.0
        self.gripper_rotate_pos = 0.0

    def send_arm_command(self, shoulder_pos, elbow_pos, rotate_pos, duration=0.5):
        self.shoulder_pos = float(shoulder_pos)
        self.elbow_pos = float(elbow_pos)
        self.gripper_rotate_pos = float(rotate_pos)
        
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.shoulder_pos, self.elbow_pos, self.gripper_rotate_pos]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)

    def send_gripper_command(self, opening_distance, duration=0.5):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [-opening_distance, -opening_distance]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points.append(point)
        self.gripper_pub.publish(traj)

    def stop_movement(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def move_backward(self):
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def rotate_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def rotate_right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

# Gestion du terminal
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    node = KeyboardRobotController()
    
    print(msg)
    
    try:
        while True:
            key = get_key()
            
            # --- Mouvement de la base ---
            if key == 'z' or key == 'w':
                node.move_forward()
            
            elif key == 's':
                node.move_backward()
            
            elif key == 'q' or key == 'a':
                node.rotate_left()
            
            elif key == 'd':
                node.rotate_right()
            
            elif key == ' ':
                node.stop_movement()
            
            # --- Bras ---
            elif key == '1':
                print("Bras: Position étendue")
                node.send_arm_command(0.75, 0.75, node.gripper_rotate_pos)
            
            elif key == '2':
                print("Bras: Position rangée")
                node.send_arm_command(-1.0, 3.14, node.gripper_rotate_pos)
            
            # --- Pince ---
            elif key == '3':
                print("Pince: Fermeture")
                node.send_gripper_command(0.0)
            
            elif key == '4':
                print("Pince: Ouverture")
                node.send_gripper_command(0.12)
            
            # --- Rotation pince ---
            elif key == 'e':
                print("Pince: Rotation anti-horaire")
                node.gripper_rotate_pos = min(node.gripper_rotate_pos + 0.2, 3.14159)
                node.send_arm_command(node.shoulder_pos, node.elbow_pos, node.gripper_rotate_pos)
            
            elif key == 'r':
                print("Pince: Rotation horaire")
                node.gripper_rotate_pos = max(node.gripper_rotate_pos - 0.2, -3.14159)
                node.send_arm_command(node.shoulder_pos, node.elbow_pos, node.gripper_rotate_pos)

    except KeyboardInterrupt:
        print("\nFermeture...")
        node.stop_movement()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
